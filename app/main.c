#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "telemetry.h"
#include "connect.h"

#define SSID "-"
#define PASS "-"
#define SERVER_IP "192.168.1.100"
#define SERVER_PORT 8080
#define MQTT_TOPIC "test/topic"

#define TASK_WIFI_PRIORITY tskIDLE_PRIORITY + 1UL
#define TASK_DUMMY_PRIORITY tskIDLE_PRIORITY + 2UL

typedef struct {
    uint8_t id;
    uint8_t data;
} TelemetryData;

QueueHandle_t xTelemetryQueue = NULL;
TaskHandle_t xTaskConnect;
TaskHandle_t xTaskMqtt;
TaskHandle_t xTaskDummy;

mqtt_config_t mqtt_config;

void mqtt_publish_task(__unused void *params) {
    TelemetryData telemetry;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        mqtt_config = mqtt_setup(SERVER_IP, SERVER_PORT, MQTT_TOPIC, 1);
          
        printf("[INFO] Attempting to connect to broker at %s:%d\n", SERVER_IP, SERVER_PORT);
        mqtt_connect(&mqtt_config);
        
        while (conn_status() == 1 || conn_status() == 3) {
            if (xQueueReceive(xTelemetryQueue, &telemetry, pdMS_TO_TICKS(500)) == pdPASS) {
                printf("[INFO] Retreived {id: %u, data: %u} from queue\n", telemetry.id, telemetry.data);

                // basic serializer for demonstration purposes only.
                char serialized_data[4];
                sprintf(serialized_data, "%u%u", telemetry.id, telemetry.data);

                mqtt_pub(&mqtt_config, &serialized_data, sizeof(serialized_data));
            } else {
                printf("[WARN] Queue is empty or timeout occurred\n");
            }

            vTaskDelay(pdMS_TO_TICKS(10000));
        }
        
        xTaskNotifyGive(xTaskConnect);
    }
}

void connect_task(__unused void *params) {
    bool reconn = 0;
    while (1) {
        if(connect(SSID, PASS, reconn, 1)) {
            reconn = 1;
            continue;
        } else {
            xTaskNotifyGive(xTaskMqtt);
        }
        
        reconn = 1;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

void dummy_task(__unused void *params) {
    TelemetryData telemetry;
    uint8_t counter = 0;
    
    while (1) {
        telemetry.id = 1;
        telemetry.data = counter++;
        
        if (xQueueSend(xTelemetryQueue, &telemetry, pdMS_TO_TICKS(100)) == pdPASS) {
            printf("[INFO] Sent {id: %u, data: %u} to queue\n", telemetry.id, telemetry.data);
        } else {
            printf("[ERROR] Failed to send message\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void vLaunch( void) {
    xTaskCreate(dummy_task, "DummyThread", configMINIMAL_STACK_SIZE, NULL, TASK_DUMMY_PRIORITY, &xTaskDummy);
    xTaskCreate(connect_task, "ConnectThread", configMINIMAL_STACK_SIZE, NULL, TASK_WIFI_PRIORITY, &xTaskConnect);
    xTaskCreate(mqtt_publish_task, "MQTTPubThread", configMINIMAL_STACK_SIZE, NULL, TASK_WIFI_PRIORITY, &xTaskMqtt);
    vTaskStartScheduler();
}

int main(void) {
    stdio_init_all();
    xTelemetryQueue = xQueueCreate(10, sizeof(TelemetryData));
    vLaunch();
}