#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "connect.h"

int connect(const char *ssid, const char *pw, bool reconn, bool log) {
    int res;
    
    if (reconn) {
        res = cyw43_arch_init();
        if (res) {
            if (log) printf("[ERROR] Failed to initialise\n");
            return res;
        }
        cyw43_arch_enable_sta_mode();
    }
    
    if (log) printf("[INFO] Attempting to connect to wireless endpoint: %s\n", ssid);
    
    res = cyw43_arch_wifi_connect_timeout_ms(ssid, pw, CYW43_AUTH_WPA2_MIXED_PSK, 30000); 
    if (res) {
        if (log) printf("[ERROR] Failed to establish Wi-Fi connection\n");
        return res;
    } else {
        if (log) printf("[INFO] Connected to %s with IP: %s\n", ssid, ip4addr_ntoa(&netif_default->ip_addr));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        return res;
    }
}

// See CYW43_LINK_ in cyw43.h for status codes.
int conn_status() {
    return cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA); 
}

void shutdown_wifi(bool log) {
    if (log) printf("[INFO] Shutting down cyw43\n");
    cyw43_arch_disable_sta_mode();
    cyw43_arch_deinit();

    if (log) printf("[INFO] cyw43 shutdown successfully\n");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
}