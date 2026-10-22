#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "telemetry.h"

mqtt_client_t *mqtt_client;

void mqtt_publish_task(__unused void *params) {
    //int status = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA); 
    //printf("WiFi status: %d (CYW43_LINK_UP=%d)\n", status, CYW43_LINK_UP);

    mqtt_client = mqtt_client_new();
    mqtt_setup(mqtt_client, true);

    printf("[INFO] Attempting to connect to broker at %s:%d\n", SERVER_IP, SERVER_PORT);
    mqtt_connect(mqtt_client, true);
    
    char payload[] = "Hello";
    uint8_t payload_len = sizeof(payload) - 1; 

    while (1) {
        mqtt_pub(mqtt_client, payload, payload_len, true);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    vTaskDelete(NULL);
}

void connect_task(__unused void *params) {
    if (cyw43_arch_init()) {
        printf("[ERROR] Failed to initialise\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    printf("[INFO] Attempting to connect to wireless endpoint: %s\n", SSID);

    if (cyw43_arch_wifi_connect_timeout_ms(SSID, PASS, CYW43_AUTH_WPA2_MIXED_PSK, 30000)) {
        printf("[ERROR] Failed to establish Wi-Fi connection\n");
        exit(1);
    } else {
        printf("[INFO] Connected to %s with IP: %s\n", SSID, ip4addr_ntoa(&netif_default->ip_addr));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        xTaskCreate(mqtt_publish_task, "MQTTPubThread", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1UL, NULL);
    }

    vTaskDelete(NULL);

    /*while (true) {
        vTaskDelay(10000);
    }

    cyw43_arch_disable_sta_mode();
    cyw43_arch_deinit();*/
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