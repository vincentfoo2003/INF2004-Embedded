#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "telemetry.h"
#include "connect.h"
#include "motor_pid_driver.h"

#define SSID        "-"
#define PASS        "-"
#define SERVER_IP   "192.168.1.100"
#define SERVER_PORT 1883U
#define MQTT_TOPIC  "se17/telemetry"
#define LOG         true

#define TASK_WIFI_PRIORITY    (tskIDLE_PRIORITY + 1UL)
#define TASK_CONTROL_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define TASK_MOTOR_PRIORITY   (tskIDLE_PRIORITY + 3UL)

typedef struct {
    uint8_t  id;
    uint32_t data;
} TelemetryData;

volatile uint8_t g_system_faults = 0U;
QueueHandle_t    g_motor_queue   = NULL;
QueueHandle_t    xTelemetryQueue = NULL;

TaskHandle_t xTaskConnect  = NULL;
TaskHandle_t xTaskMqttPub  = NULL;
TaskHandle_t xTaskMqttConn = NULL;
TaskHandle_t xTaskMotor    = NULL;

mqtt_config_t mqtt_config;

/**
 * @brief Overrides weak hook in motor_pid_driver.c to forward motor telemetry to MQTT queue.
 */
bool mqtt_enqueue_telemetry(uint8_t source_id, uint32_t packed_data)
{
    if (NULL == xTelemetryQueue)
    {
        return false;
    }

    TelemetryData item = {
        .id   = source_id,
        .data = packed_data
    };

    /* Non-blocking send so the 50Hz PID loop never stalls when Wi-Fi is busy */
    return (pdPASS == xQueueSend(xTelemetryQueue, &item, (TickType_t)0U));
}

void mqtt_publish_task(void *params)
{
    (void)params;
    TelemetryData telemetry;
    char serialized_data[24];

    for (;;)
    {
        /* Block until MQTT connection task signals that broker is connected */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Drain and publish telemetry items while MQTT remains connected */
        while (mqtt_status(&mqtt_config) != 0U)
        {
            if (xQueueReceive(xTelemetryQueue, &telemetry, pdMS_TO_TICKS(500U)) == pdPASS)
            {
                int len = snprintf(serialized_data, sizeof(serialized_data),
                                   "%u:%lu", telemetry.id, (unsigned long)telemetry.data);
                if (len > 0)
                {
                    mqtt_pub(&mqtt_config, serialized_data, (u16_t)len);
                }
            }
        }
    }
}

void mqtt_conn_task(void *params)
{
    (void)params;
    bool setup = false;

    for (;;)
    {
        if (conn_status() <= 1)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        if (!setup)
        {
            mqtt_config = mqtt_setup(SERVER_IP, SERVER_PORT, MQTT_TOPIC, LOG);
            setup = true;
        }

        mqtt_connect(&mqtt_config);
        vTaskDelay(pdMS_TO_TICKS(2000U));

        if (mqtt_status(&mqtt_config) == 0U)
        {
            mqtt_wifi_disconnect(&mqtt_config);
            vTaskDelay(pdMS_TO_TICKS(3000U));
            continue;
        }

        /* Notify publish task to start streaming queue data */
        xTaskNotifyGive(xTaskMqttPub);

        /* Stay connected as long as Wi-Fi and MQTT broker remain up */
        while ((conn_status() > 1) && (mqtt_status(&mqtt_config) != 0U))
        {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }

        mqtt_wifi_disconnect(&mqtt_config);
    }
}

void connect_task(void *params)
{
    (void)params;
    bool reconn = false;

    for (;;)
    {
        if (connect(SSID, PASS, reconn, LOG))
        {
            reconn = true;
            vTaskDelay(pdMS_TO_TICKS(2000U));
            continue;
        }
        else
        {
            xTaskNotifyGive(xTaskMqttConn);
        }

        reconn = true;

        while (conn_status() > 1)
        {
            conn_poll();
            vTaskDelay(pdMS_TO_TICKS(5000U));
        }
    }
}

void vLaunch(void)
{
    xTaskCreate(vMotionControlTask, "MotionPID", 1024U, NULL, TASK_MOTOR_PRIORITY, &xTaskMotor);
    xTaskCreate(connect_task, "ConnectThread", configMINIMAL_STACK_SIZE, NULL, TASK_WIFI_PRIORITY, &xTaskConnect);
    xTaskCreate(mqtt_conn_task, "MQTTConnThread", configMINIMAL_STACK_SIZE, NULL, TASK_WIFI_PRIORITY, &xTaskMqttConn);
    xTaskCreate(mqtt_publish_task, "MQTTPubThread", configMINIMAL_STACK_SIZE, NULL, TASK_WIFI_PRIORITY, &xTaskMqttPub);
    vTaskStartScheduler();
}

int main(void)
{
    stdio_init_all();

    g_motor_queue   = xQueueCreate(8U, sizeof(uint16_t));
    xTelemetryQueue = xQueueCreate(32U, sizeof(TelemetryData));

    vLaunch();

    for (;;)
    {
    }
    return 0;
}