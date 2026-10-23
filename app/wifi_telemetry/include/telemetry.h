#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "lwip/apps/mqtt.h"

struct mqtt_config_s {
    mqtt_client_t *client;
    const char *ip;
    uint16_t port;
    const char *topic;
    bool log;
} typedef mqtt_config_t;

mqtt_config_t mqtt_setup(const char *ip, uint16_t port, const char *topic, bool log);
void mqtt_connect(mqtt_config_t *config);
void mqtt_pub(mqtt_config_t *config, const void *payload, u16_t payload_len);

#endif