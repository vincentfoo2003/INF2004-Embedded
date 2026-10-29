#ifndef CONNECT_H
#define CONNECT_H

int connect(const char *ssid, const char *pw, bool reconn, bool log);
int conn_status();
void conn_poll();
void shutdown_wifi(bool log);

#endif