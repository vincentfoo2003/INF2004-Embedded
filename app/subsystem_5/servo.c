/*
 * Ultrasonic Obstacle Profiling (Buddy 5 / Subsystem 5) - servo PWM driver.
 *
 * Hardware-coupled servo driver for GP12. Configures GP12's own PWM slice for a
 * 20 ms (50 Hz) frame and positions the servo by high-pulse width.
 *
 * Design mapping (design.md "Servo angle -> PWM duty"):
 *   - 50 Hz servo => 20 ms (20000 us) period.
 *   - Position is set by the high pulse: ~1000 us (one extreme) ..
 *     ~2000 us (other extreme), ~1500 us centre.
 *   - The usable 30..150 deg scan span maps linearly onto 1000..2000 us:
 *         pulse_us = 1000 + ((angle_deg - 30) * 1000) / 120
 *   - level (slice counts) = pulse_us * wrap / 20000.
 *
 * Clock/wrap: the slice clock is clk_sys / clkdiv. We pick clkdiv so one slice
 * tick == 1 us (clkdiv = clk_sys_hz / 1_000_000), giving a 1 us resolution and
 * wrap = SRV_PERIOD_US - 1 = 19999 for the full 20 ms frame. Computing clkdiv
 * from the live clk_sys makes the 20 ms period correct regardless of the actual
 * system clock.
 *
 * The clamp decision is the PURE servo_clamp_angle() in obstacle_logic.c so the
 * "servo angle bounded" property is host-testable; this unit owns only the PWM
 * hardware and the FAULT_SERVO latch.
 *
 * Conventions: Barr-C style - snake_case, #define macros.
 */

#include "servo.h"

#include "obstacle_avoidance.h"    /* FAULT_SERVO */
#include "obstacle_integration.h"  /* SRV_PWM_PIN, SRV_PWM_HZ, SRV_MIN/MAX_DEG */
#include "obstacle_logic.h"        /* servo_clamp_angle (pure) */

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

/* ----- Derived servo PWM timing constants ----- */

/* 50 Hz => 20 ms frame. Expressed in us so the pulse maths stays integer. */
#define SRV_PERIOD_US   (1000000u / (uint32_t)SRV_PWM_HZ) /* 20000 us */

/* One slice tick == 1 us; wrap counts the whole 20 ms frame (0..19999). */
#define SRV_WRAP        ((uint16_t)(SRV_PERIOD_US - 1u))   /* 19999 */

/* Pulse-width endpoints for the usable scan span (us). */
#define SRV_PULSE_MIN_US  1000u  /* at SRV_MIN_DEG (30 deg)  */
#define SRV_PULSE_MAX_US  2000u  /* at SRV_MAX_DEG (150 deg) */
#define SRV_PULSE_SPAN_US (SRV_PULSE_MAX_US - SRV_PULSE_MIN_US) /* 1000 us */

/* Angular span mapped onto the pulse span: SRV_MAX_DEG - SRV_MIN_DEG (120). */
#define SRV_DEG_SPAN      ((uint32_t)SRV_MAX_DEG - (uint32_t)SRV_MIN_DEG)

/* FAULT_SERVO latch for this driver; ORed into the central mask by task 12.2. */
static bool s_servo_fault = false;

/* clk_sys counts per PWM tick so each tick is ~1 us. Cached at init. */
static uint32_t s_clkdiv_int = 1u;

/*
 * Linear map of an in-range angle (30..150 deg) to its high-pulse width in us
 * (1000..2000). Pure arithmetic; kept local to the driver because the result is
 * a hardware pulse width, not a reusable logic primitive.
 */
static uint16_t servo_angle_to_pulse_us(uint8_t angle_deg)
{
    uint32_t pulse_us;

    pulse_us = (uint32_t)SRV_PULSE_MIN_US
             + (((uint32_t)angle_deg - (uint32_t)SRV_MIN_DEG)
                * (uint32_t)SRV_PULSE_SPAN_US) / SRV_DEG_SPAN;

    return (uint16_t)pulse_us;
}

void servo_init(void)
{
    uint32_t sys_hz;
    uint32_t slice_num;

    s_servo_fault = false;

    /* Route GP12 to its PWM function and grab its dedicated slice. */
    gpio_set_function(SRV_PWM_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(SRV_PWM_PIN);

    /* Pick an integer clkdiv so one tick ~= 1 us (clk_sys / 1 MHz). Computing it
     * from the live system clock keeps the 20 ms frame correct on any clk_sys.
     * Guard against a zero divisor on an unexpectedly low clock. */
    sys_hz = clock_get_hz(clk_sys);
    s_clkdiv_int = sys_hz / 1000000u;
    if (s_clkdiv_int == 0u) {
        s_clkdiv_int = 1u;
    }

    pwm_set_clkdiv_int_frac(slice_num, (uint8_t)s_clkdiv_int, 0u);
    pwm_set_wrap(slice_num, SRV_WRAP);
    pwm_set_gpio_level(SRV_PWM_PIN, 0u); /* no pulse until first commanded angle */
    pwm_set_enabled(slice_num, true);
}

void servo_set_angle(uint8_t angle_deg)
{
    bool     out_of_range = false;
    uint8_t  clamped;
    uint16_t pulse_us;
    uint32_t level;

    /* Pure clamp: always returns an angle in [30,150] and tells us if the
     * request was outside the range so we can latch FAULT_SERVO. */
    clamped = servo_clamp_angle(angle_deg, &out_of_range);
    if (out_of_range) {
        s_servo_fault = true; /* clamp to nearest bound AND flag the fault */
    }

    pulse_us = servo_angle_to_pulse_us(clamped);

    /* level = pulse_us * wrap / 20000 (us). With wrap == period_us - 1 this is
     * essentially the pulse width in ticks. 32-bit math avoids overflow
     * (2000 * 19999 fits in uint32). */
    level = ((uint32_t)pulse_us * (uint32_t)SRV_WRAP) / SRV_PERIOD_US;

    pwm_set_gpio_level(SRV_PWM_PIN, (uint16_t)level);
}

bool servo_fault(void)
{
    return s_servo_fault;
}
