#ifndef HCSR04_H
#define HCSR04_H

/*
 * Ultrasonic Obstacle Profiling (Buddy 5 / Subsystem 5) - HC-SR04 driver header.
 *
 * This is the HC-SR04 ultrasonic sensor driver translation unit (one TU per
 * driver, matching the repo convention). It owns the GP16 TRIG output and the
 * GP17 ECHO input and turns a ranging cycle into an echo high-time. It is the
 * ONLY place the HC-SR04 pins are touched.
 *
 * Hardware calls live here (hardware/gpio.h, pico/time.h); the pure cm
 * conversion stays in obstacle_logic.c (us_echo_to_cm) so the Correctness
 * Properties remain host-testable off-device.
 *
 * Multiple HC-SR04 tasks share this TU:
 *   - task 8.1: us_pins_init(), us_trigger()        (this file)
 *   - task 8.2: us_measure_echo_us()                (APPENDS here)
 * Each task APPENDS its own declaration here and definition in hcsr04.c; do not
 * clobber siblings.
 *
 * Pin map and timing constants (US_TRIG_PIN 16, US_ECHO_PIN 17,
 * US_TRIG_PULSE_US 10, ...) are the single source of truth in
 * obstacle_integration.h and are shared by this driver.
 *
 * Conventions: Barr-C style - snake_case functions, #define macros, header
 * guards.
 *
 * Hardware prerequisite (wiring note, not code): HC-SR04 ECHO outputs 5 V but
 * RP2040 GPIO is 3.3 V tolerant only - a resistor voltage divider (or level
 * shifter) on the ECHO line is MANDATORY before GP17.
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_integration.h"   /* US_TRIG_PIN / US_ECHO_PIN / US_TRIG_PULSE_US */

/* --------------------------------------------------------------------------
 * Task 8.1: pin init + trigger pulse
 * --------------------------------------------------------------------------
 * us_pins_init() configures the two HC-SR04 lines once at startup:
 *   - US_TRIG_PIN (GP16) as a GPIO output, driven low initially.
 *   - US_ECHO_PIN (GP17) as a GPIO input (fed via the mandatory 5V->3.3V
 *     divider on the wiring side).
 * Must be called before us_trigger() / us_measure_echo_us().
 */
void us_pins_init(void);

/* Emit one HC-SR04 trigger pulse on US_TRIG_PIN: drive a clean low, settle for
 * 2 us, raise high for US_TRIG_PULSE_US (>= 10 us), then return low. Starts a
 * single ranging cycle; the caller measures ECHO afterwards. Assumes
 * us_pins_init() has already run. */
void us_trigger(void);

/* --------------------------------------------------------------------------
 * Task 8.2: measure the echo high-time by polling (NO GPIO IRQ)
 * --------------------------------------------------------------------------
 * us_measure_echo_us() runs one full ranging cycle and returns the ECHO high
 * pulse width in microseconds:
 *   1. fire us_trigger() to start a cycle,
 *   2. poll US_ECHO_PIN for the rising edge (0 -> 1), and
 *   3. time the high phase (1 -> 0), returning that duration in us.
 *
 * Both phases are bounded by US_ECHO_TIMEOUT_US (25000 us, ~430 cm round-trip):
 * if the rising edge never arrives, or if the line is stuck high past the
 * timeout, the function returns 0 to signal NO_ECHO. The caller converts the
 * returned microseconds to centimetres with us_echo_to_cm() (0 -> NO_ECHO).
 *
 * Measurement is pure polling on time_us_32(); this function MUST NOT call
 * gpio_set_irq_enabled_with_callback - the single master GPIO ISR belongs to
 * Buddy 2. Assumes us_pins_init() has already run.
 */
uint32_t us_measure_echo_us(void);

#endif /* HCSR04_H */
