#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "lwip/apps/mqtt.h"

#define SSID "-"
#define PASS "-"
#define SERVER_IP "192.168.1.100"
#define SERVER_PORT 8080
#define MQTT_TOPIC "test/topic"

void mqtt_setup(mqtt_client_t *client, bool log);
void mqtt_connect(mqtt_client_t *client, bool log);
void mqtt_pub(mqtt_client_t *client, const void *payload, u16_t payload_len, bool log);

#endif