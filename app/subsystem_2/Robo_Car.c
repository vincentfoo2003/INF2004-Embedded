#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "motor_control.h" 

int main() {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("Serial Monitor Connected. Starting routine...\n");

    // Call functions from motor_control.c
    setup_motor();
    setup_encoder();

    while (1) {
        // encoder_ticks is available because of the extern declaration in the .h file
        printf("Current Ticks: %d\n", encoder_ticks);
        
        set_motor_state(true, 32768);
        sleep_ms(2000);
        
        set_motor_state(false, 32768);
        sleep_ms(2000);
    }
}