#include "pico/stdlib.h"
#include "telemetry.h"
#include <string.h>

void mqtt_pub_request_cb(void *arg, err_t result) {
    if (result == ERR_OK) {
        printf("[INFO] Published to broker %s:%d\n", SERVER_IP, SERVER_PORT);
    }
}

void mqtt_sub_request_cb(void *arg, err_t result) {
    if (result == ERR_OK) {
        printf("[INFO] Subscribed to topic: %s\n", MQTT_TOPIC);
    }
}

void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    printf("[INFO] Topic: %s (len: %lu)\n", topic, tot_len);
}

void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    printf("[INFO] Data: %.*s\n", len, (char *)data);
}

void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[INFO] Connected to broker %s:%d\n", SERVER_IP, SERVER_PORT);
        mqtt_subscribe(client, MQTT_TOPIC, 0, mqtt_sub_request_cb, NULL);
    } else {
        printf("[ERROR] MQTT connection failed: %d\n", status);
    }
}

void mqtt_setup(mqtt_client_t *client, bool log) {
    if (log) {
        mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
    } else {
        mqtt_set_inpub_callback(client, NULL, NULL, NULL);
    } 
}

void mqtt_connect(mqtt_client_t *client, bool log) {
    ip_addr_t broker_ip;
    ipaddr_aton(SERVER_IP, &broker_ip);
     
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico_w";
    ci.keep_alive = 60;

    if (log) {
        mqtt_client_connect(client, &broker_ip, SERVER_PORT, 
            mqtt_connection_cb, NULL, &ci);
    } else {
        mqtt_client_connect(client, &broker_ip, SERVER_PORT, 
            NULL, NULL, &ci);
    }
}

void mqtt_pub(mqtt_client_t *client, const void *payload, u16_t payload_len, bool log) {
    if (log) {
        mqtt_publish(client, MQTT_TOPIC, payload, payload_len, 0, 0, 
            mqtt_pub_request_cb, NULL);
    } else {
        mqtt_publish(client, MQTT_TOPIC, payload, payload_len, 0, 0, 
            NULL, NULL);
    }
}