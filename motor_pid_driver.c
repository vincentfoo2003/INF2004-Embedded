#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// RP2040 PWM GPIO function selector, for SDK/header versions that omit the name.
#ifndef GPIO_FUNC_PWM
#define GPIO_FUNC_PWM 4
#endif

// Rising-edge interrupt event mask for SDK/header versions that omit the name.
#ifndef GPIO_IRQ_EDGE_RISE
#define GPIO_IRQ_EDGE_RISE 0x4u
#endif

// Pin Mappings
#define ENCODER_A_PIN 0
#define ENCODER_B_PIN 1
#define MOTOR_ENA_PIN 2
#define MOTOR_IN1_PIN 3
#define MOTOR_IN2_PIN 4

// Volatile tick counter for interrupt safety
volatile int encoder_ticks = 0;

// Hardware interrupt service routine for the encoder
void encoder_callback(unsigned int gpio, unsigned int events) {
    if (gpio == ENCODER_A_PIN) {
        // Read phase B to determine the direction of rotation
        if (gpio_get(ENCODER_B_PIN)) {
            encoder_ticks++; // Forward
        } else {
            encoder_ticks--; // Reverse
        }
    }
}

void setup_motor() {
    // Initialize direction control pins
    gpio_init(MOTOR_IN1_PIN);
    gpio_set_dir(MOTOR_IN1_PIN, true);
    gpio_init(MOTOR_IN2_PIN);
    gpio_set_dir(MOTOR_IN2_PIN, true);

    // Initialize ENA pin for hardware PWM
    gpio_set_function(MOTOR_ENA_PIN, GPIO_FUNC_PWM);
    unsigned int slice_num = pwm_gpio_to_slice_num(MOTOR_ENA_PIN);
    
    // Configure PWM wrap for 16-bit resolution (0 to 65535)
    pwm_set_wrap(slice_num, 65535);
    
    // Start with motor off (0 duty cycle)
    pwm_set_gpio_level(MOTOR_ENA_PIN, 0); 
    pwm_set_enabled(slice_num, true);
}

void setup_encoder() {
    // Initialize encoder inputs with pull-ups
    gpio_init(ENCODER_A_PIN);
    gpio_set_dir(ENCODER_A_PIN, true);
    gpio_pull_up(ENCODER_A_PIN);
    
    gpio_init(ENCODER_B_PIN);
    gpio_set_dir(ENCODER_B_PIN, true);
    gpio_pull_up(ENCODER_B_PIN);

    // Trigger an interrupt on the rising edge of Phase A
    gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_EDGE_RISE, true, &encoder_callback);
}

// Helper function to drive the motor
// Speed parameter accepts 0 (stop) to 65535 (max speed)
void set_motor_state(bool forward, unsigned int speed) {
    if (forward) {
        gpio_put(MOTOR_IN1_PIN, 1);
        gpio_put(MOTOR_IN2_PIN, 0);
    } else {
        gpio_put(MOTOR_IN1_PIN, 0);
        gpio_put(MOTOR_IN2_PIN, 1);
    }
    pwm_set_gpio_level(MOTOR_ENA_PIN, speed);
}

int main() {
    stdio_init_all();

    // Wait for the serial monitor to open before starting
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("Serial Monitor Connected. Starting routine...\n");

    setup_motor();
    setup_encoder();

    while (1) {
        printf("Current Ticks: %d\n", encoder_ticks);
        
        // Spin forward at 50% speed for 2 seconds
        set_motor_state(true, 32768);
        sleep_ms(2000);
        
        // Spin backward at 50% speed for 2 seconds
        set_motor_state(false, 32768);
        sleep_ms(2000);
    }
}