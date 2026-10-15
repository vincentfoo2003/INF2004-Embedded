#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

// Declare the functions so main.c can use them
void setup_motor(void);
void setup_encoder(void);
void set_motor_state(bool forward, unsigned int speed);

// Make the tick counter accessible to main.c without redefining it
extern volatile int32_t encoder_ticks;

#endif // MOTOR_CONTROL_H