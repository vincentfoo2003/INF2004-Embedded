#ifndef SERVO_H
#define SERVO_H

/*
 * Ultrasonic Obstacle Profiling (Buddy 5 / Subsystem 5) - servo PWM driver.
 *
 * Hardware driver for the ranging servo on GP12 (its own dedicated PWM slice,
 * independent of the motor slices, so there is no slice contention). Drives a
 * 20 ms / 50 Hz frame and positions the servo by high-pulse width, mapping the
 * usable 30..150 deg scan span onto a ~1000..2000 us pulse.
 *
 * One translation unit per driver (servo.c). The angle-clamp decision lives in
 * the PURE obstacle_logic unit (servo_clamp_angle) so it is host-testable; this
 * driver owns only the hardware (PWM config + level) and the FAULT_SERVO latch.
 *
 * Conventions: Barr-C style - snake_case functions, #define macros, header
 * guards.
 */

#include <stdbool.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
 * Driver lifecycle
 * -------------------------------------------------------------------------- */

/* Configure GP12's dedicated PWM slice for a 20 ms (50 Hz) servo frame and
 * enable it. Clears the servo fault latch. Call once before servo_set_angle. */
void servo_init(void);

/* Command the servo to angle_deg. The angle is clamped to
 * [SRV_MIN_DEG, SRV_MAX_DEG]; when the request is outside that range it is
 * clamped to the nearest bound AND the FAULT_SERVO latch is set (readable via
 * servo_fault()). The clamped angle is linearly mapped 30..150 deg ->
 * ~1000..2000 us pulse and applied with pwm_set_gpio_level. */
void servo_set_angle(uint8_t angle_deg);

/* True once an out-of-range angle has been commanded since servo_init(). The
 * central fault aggregator (task 12.2) ORs this into FAULT_SERVO. */
bool servo_fault(void);

#endif /* SERVO_H */
