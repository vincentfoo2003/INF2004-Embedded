/*
 * Ultrasonic Obstacle Profiling (Buddy 5 / Subsystem 5) - HC-SR04 driver.
 *
 * Hardware-coupled HC-SR04 driver: owns the GP16 TRIG output and GP17 ECHO
 * input and drives a single ranging cycle. This is the one translation unit
 * that touches the ultrasonic pins; the pure cm conversion (us_echo_to_cm)
 * stays in obstacle_logic.c so it remains host-testable.
 *
 * Multiple HC-SR04 tasks APPEND to this TU (do not clobber siblings):
 *   - task 8.1: us_pins_init(), us_trigger()     (below)
 *   - task 8.2: us_measure_echo_us()             (appended later)
 *
 * NOTE: echo is measured by polling + time_us_32() (task 8.2); this driver
 * MUST NOT call gpio_set_irq_enabled_with_callback - the single master GPIO
 * ISR is owned by Buddy 2.
 *
 * Conventions: Barr-C style - snake_case, #define macros in the shared headers.
 */

#include "hcsr04.h"

#include "hardware/gpio.h"   /* gpio_init / gpio_set_dir / gpio_put */
#include "pico/time.h"       /* busy_wait_us */

/* Settle time (us) to guarantee a clean low on TRIG before the start pulse, so
 * the HC-SR04 sees a crisp rising edge rather than a glitchy one. */
#define US_TRIG_SETTLE_US  2u

/* ==========================================================================
 * Task 8.1: us_pins_init
 * --------------------------------------------------------------------------
 * One-time pin setup. Configure GP16 (US_TRIG_PIN) as a GPIO output driven low
 * and GP17 (US_ECHO_PIN) as a GPIO input. Call once at startup before any
 * trigger/measure.
 * _Requirements: 1.12, 5.3_
 * ========================================================================== */
void us_pins_init(void)
{
    /* TRIG (GP16): GPIO output, start low so the first us_trigger() sees a
     * clean idle line. */
    gpio_init(US_TRIG_PIN);
    gpio_set_dir(US_TRIG_PIN, GPIO_OUT);
    gpio_put(US_TRIG_PIN, 0);

    /* ECHO (GP17): GPIO input. The 5V ECHO signal reaches GP17 through the
     * mandatory external 5V->3.3V divider (wiring prerequisite, not code). */
    gpio_init(US_ECHO_PIN);
    gpio_set_dir(US_ECHO_PIN, GPIO_IN);
}

/* ==========================================================================
 * Task 8.1: us_trigger
 * --------------------------------------------------------------------------
 * Emit one HC-SR04 start pulse: drive a clean low, settle 2 us, raise high for
 * US_TRIG_PULSE_US (>= 10 us), then return low. busy_wait_us keeps the pulse
 * tight and deterministic; the whole routine runs only while the car is
 * stopped, so the short busy-wait steals no time from motion/safety.
 * _Requirements: 1.12, 5.3_
 * ========================================================================== */
void us_trigger(void)
{
    gpio_put(US_TRIG_PIN, 0);            /* ensure a clean low before the pulse */
    busy_wait_us(US_TRIG_SETTLE_US);     /* 2 us settle */

    gpio_put(US_TRIG_PIN, 1);            /* start the ranging pulse */
    busy_wait_us(US_TRIG_PULSE_US);      /* hold high >= 10 us */

    gpio_put(US_TRIG_PIN, 0);            /* return low; pulse complete */
}

/* ==========================================================================
 * Task 8.2: us_measure_echo_us
 * --------------------------------------------------------------------------
 * Run one ranging cycle and return the ECHO high-time in microseconds by
 * polling + time_us_32(). Deliberately NO GPIO IRQ: the single master GPIO ISR
 * is owned by Buddy 2, so this driver MUST NOT call
 * gpio_set_irq_enabled_with_callback.
 *
 * Sequence:
 *   1. us_trigger() starts the cycle.
 *   2. Poll for the rising edge (ECHO 0 -> 1). If it does not arrive within
 *      US_ECHO_TIMEOUT_US (25000 us), return 0 (NO_ECHO).
 *   3. Time the high phase (ECHO 1 -> 0). The same timeout guards a line that
 *      is stuck high; on timeout return 0 (NO_ECHO).
 *   4. Otherwise return (now - rise), the high duration in microseconds.
 *
 * Returning 0 funnels into us_echo_to_cm()'s NO_ECHO (255) path off-device.
 * time_us_32() wraps every ~71.6 min; the unsigned subtractions below stay
 * correct across a single wrap because they are computed modulo 2^32, and each
 * measured interval is far shorter than the wrap period.
 * _Requirements: 1.4, 3.6, 5.3_
 * ========================================================================== */
uint32_t us_measure_echo_us(void)
{
    uint32_t start;
    uint32_t rise;

    /* Kick off a single ranging cycle on TRIG. */
    us_trigger();

    /* Phase 1: wait for the rising edge (ECHO goes high). Bail out with
     * NO_ECHO (0) if it never arrives within the timeout window. */
    start = time_us_32();
    while (0 == gpio_get(US_ECHO_PIN))
    {
        if ((time_us_32() - start) > US_ECHO_TIMEOUT_US)
        {
            return 0u;   /* no rising edge -> NO_ECHO */
        }
    }

    /* Phase 2: ECHO is high; time how long it stays high. The same timeout
     * guards a stuck-high line (e.g. missing/broken divider). */
    rise = time_us_32();
    while (0 != gpio_get(US_ECHO_PIN))
    {
        if ((time_us_32() - rise) > US_ECHO_TIMEOUT_US)
        {
            return 0u;   /* stuck high -> NO_ECHO */
        }
    }

    /* Falling edge seen: high-time in microseconds. */
    return (time_us_32() - rise);
}
