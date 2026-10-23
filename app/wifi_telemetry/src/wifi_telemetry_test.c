#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "telemetry.h"
#include "connect.h"

#define SSID "-"
#define PASS "-"
#define SERVER_IP "192.168.1.100"
#define SERVER_PORT 8080
#define MQTT_TOPIC "test/topic"

mqtt_config_t config;

void mqtt_publish_task(__unused void *params) {
    //int status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA); 
    //printf("WiFi status: %d (CYW43_LINK_UP=%d)\n", status, CYW43_LINK_UP);

    config = mqtt_setup(SERVER_IP, SERVER_PORT, MQTT_TOPIC, 1);

    printf("[INFO] Attempting to connect to broker at %s:%d\n", SERVER_IP, SERVER_PORT);
    mqtt_connect(&config);
    
    char payload[] = "Hello";
    uint8_t payload_len = sizeof(payload) - 1; 

    while (1) {
        mqtt_pub(&config, payload, payload_len);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    vTaskDelete(NULL);
}

void connect_task(__unused void *params) {
    if(connect(SSID, PASS, 0, 1)) {
        exit(1);
    } else {
        xTaskCreate(mqtt_publish_task, "MQTTPubThread", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, NULL);
    }

    vTaskDelete(NULL);
}

void vLaunch( void) {
    TaskHandle_t task;
    xTaskCreate(connect_task, "MainThread", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2UL, &task);
    vTaskStartScheduler();
}

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    vLaunch();
}