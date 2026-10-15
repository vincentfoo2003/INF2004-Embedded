#include "motor_control.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"

#ifndef GPIO_FUNC_PWM
#define GPIO_FUNC_PWM 4
#endif

#ifndef GPIO_IRQ_EDGE_RISE
#define GPIO_IRQ_EDGE_RISE 0x4u
#endif

// Pin Mappings
#define ENCODER_A_PIN 0
#define ENCODER_B_PIN 1
#define MOTOR_ENA_PIN 2
#define MOTOR_IN1_PIN 3
#define MOTOR_IN2_PIN 4

// Declare the shared encoder counter once; define it in a single .c file
volatile int32_t encoder_ticks = 0;

void encoder_callback(uint gpio, uint32_t events) {
    if (gpio == ENCODER_A_PIN) {
        if (gpio_get(ENCODER_B_PIN)) {
            encoder_ticks++;
        } else {
            encoder_ticks--;
        }
    }
}

void setup_motor() {
    gpio_init(MOTOR_IN1_PIN);
    gpio_set_dir(MOTOR_IN1_PIN, true); 
    gpio_init(MOTOR_IN2_PIN);
    gpio_set_dir(MOTOR_IN2_PIN, true); 

    gpio_set_function(MOTOR_ENA_PIN, GPIO_FUNC_PWM);
    unsigned int slice_num = pwm_gpio_to_slice_num(MOTOR_ENA_PIN);
    
    pwm_set_wrap(slice_num, 65535);
    pwm_set_gpio_level(MOTOR_ENA_PIN, 0); 
    pwm_set_enabled(slice_num, true);
}

void setup_encoder() {
    gpio_init(ENCODER_A_PIN);
    gpio_set_dir(ENCODER_A_PIN, false); 
    gpio_pull_up(ENCODER_A_PIN);
    
    gpio_init(ENCODER_B_PIN);
    gpio_set_dir(ENCODER_B_PIN, false); 
    gpio_pull_up(ENCODER_B_PIN);

    gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_EDGE_RISE, true, &encoder_callback);
}

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