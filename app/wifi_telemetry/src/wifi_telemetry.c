#include "pico/stdlib.h"
#include "telemetry.h"
#include <string.h>

void mqtt_pub_request_cb(void *arg, err_t result) {
    if (result == ERR_OK) {
        mqtt_config_t *config = (mqtt_config_t *)arg; 
        printf("[INFO] Published to broker %s:%d\n", config->ip, config->port);
    }
}

void mqtt_sub_request_cb(void *arg, err_t result) {
    if (result == ERR_OK) {
        mqtt_config_t *config = (mqtt_config_t *)arg; 
        printf("[INFO] Subscribed to topic: %s\n", config->topic);
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
        mqtt_config_t *config = (mqtt_config_t *)arg; 
        printf("[INFO] Connected to broker %s:%d\n", config->ip, config->port);
        mqtt_subscribe(client, config->topic, 0, mqtt_sub_request_cb, config);
    } else {
        printf("[ERROR] MQTT connection failed: %d\n", status);
    }
}

mqtt_config_t mqtt_setup(const char *ip, uint16_t port, const char *topic, bool log) { 
    mqtt_config_t config;
    config.client = mqtt_client_new();
    config.ip = ip;
    config.port = port;
    config.topic = topic;
    config.log = log;

    if (log) {
        mqtt_set_inpub_callback(config.client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, &config);
    } else {
        mqtt_set_inpub_callback(config.client, NULL, NULL, NULL);
    } 

    return config;
}

void mqtt_connect(mqtt_config_t *config) {
    ip_addr_t broker_ip;
    ipaddr_aton(config->ip, &broker_ip);

    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico_w";
    ci.keep_alive = 60;

    if (config->log) {
        mqtt_client_connect(config->client, &broker_ip, config->port, 
            mqtt_connection_cb, config, &ci);
    } else {
        mqtt_client_connect(config->client, &broker_ip, config->port, 
            NULL, NULL, &ci);
    }
}

void mqtt_pub(mqtt_config_t *config, const void *payload, u16_t payload_len) {
    if (config->log) {
        mqtt_publish(config->client, config->topic, payload, payload_len, 0, 0, 
            mqtt_pub_request_cb, config);
    } else {
        mqtt_publish(config->client, config->topic, payload, payload_len, 0, 0, 
            NULL, NULL);
    }
}