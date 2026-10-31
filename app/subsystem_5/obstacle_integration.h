#ifndef OBSTACLE_INTEGRATION_H
#define OBSTACLE_INTEGRATION_H

/*
 * Ultrasonic Obstacle Profiling (Buddy 5 / Subsystem 5) - integration header.
 *
 * This header holds EVERY external contract this subsystem depends on, coded as
 * clearly-labeled stub/extern declarations so the subsystem compiles STANDALONE
 * before the peer subsystems freeze their real interfaces.
 *
 * >>> NEGOTIATED PLACEHOLDERS <<<
 * Each contract below is an *assumed* interface owned by another Buddy. The
 * exact enums and struct layouts are NOT yet frozen in the repo. They are
 * reasonable placeholders explicitly labeled for negotiation. Confirm each with
 * its owning Buddy before the integration freeze.
 *
 * It also defines the subsystem's own pin map and timing/scan constants (frozen
 * Week 6), kept here so the driver .c files added in later tasks can share them.
 *
 * Conventions: Barr-C style - snake_case, #define pin macros, header guards.
 */

#include <stdbool.h>
#include <stdint.h>

/* ==========================================================================
 * (A) MotorCommandQueue payload - OWNED BY BUDDY 2.
 * --------------------------------------------------------------------------
 * NEGOTIATED PLACEHOLDER. Buddy 5 never writes GP2-GP7; it posts a 16-bit
 * packed command to Buddy 2's Motor_Queue. Assumed packing:
 *   [15:14] = command (2 bits), [13:0] = payload (speed or distance_ticks).
 * Replace with Buddy 2's published enum/struct once frozen.
 * ========================================================================== */
typedef enum {
    MOTOR_CMD_STOP         = 0u,  /* also used for EMERGENCY_STOP intent */
    MOTOR_CMD_FORWARD      = 1u,
    MOTOR_CMD_BYPASS_LEFT  = 2u,
    MOTOR_CMD_BYPASS_RIGHT = 3u
} motor_cmd_t;

/* 16-bit on the wire: [15:14]=command, [13:0]=payload (speed or distance_ticks) */
typedef struct {
    uint16_t packed;   /* build with obs_pack_motor_cmd() (see later tasks) */
} motor_command_t;

/* ==========================================================================
 * (B) Telemetry struct - OWNED BY BUDDY 1.
 * --------------------------------------------------------------------------
 * NEGOTIATED PLACEHOLDER. The agreed item is a uint8_t id plus a uint32_t data
 * field, superseding the current repo {uint8_t id; uint8_t data;} in main.c.
 * Buddy 5 bit-packs angle/distance/width into the 32-bit field.
 * Replace with Buddy 1's published struct once frozen.
 * ========================================================================== */
typedef struct {
    uint8_t  id;      /* TELEMETRY_ID_OBSTACLE identifies an ultrasonic message */
    uint32_t data;    /* bit-packed obstacle payload */
} TelemetryData32;

#define TELEMETRY_ID_OBSTACLE  0x05u   /* subsystem 5 */

/* ==========================================================================
 * (C) Line reacquisition - OWNED BY BUDDY 3 (line_sensor.c).
 * --------------------------------------------------------------------------
 * NEGOTIATED PLACEHOLDER. Buddy 5 CALLS this, does not implement it.
 * Returns true when the car is back over the line.
 * ========================================================================== */
extern bool line_detected(void);

/* ==========================================================================
 * (D) Hump flag - OWNED BY BUDDY 4 (Terrain Monitor).
 * --------------------------------------------------------------------------
 * NEGOTIATED PLACEHOLDER. Buddy 5 READS this, does not implement it.
 * Non-zero while a hump/incline/decline is active (an incline reads like a
 * wall to an HC-SR04, so suspected obstacles are ignored while active).
 * ========================================================================== */
typedef enum {
    HUMP_NONE    = 0,
    HUMP_INCLINE = 1,
    HUMP_LEVEL   = 2,
    HUMP_DECLINE = 3
} hump_state_t;

extern hump_state_t terrain_hump_state(void);

/* ==========================================================================
 * Pin map and subsystem constants - OWNED BY BUDDY 5 (frozen Week 6 Table 2).
 * --------------------------------------------------------------------------
 * These are this subsystem's own definitions (not negotiated placeholders).
 * Kept here so the driver .c files added in later tasks share one source of
 * truth.
 *
 * Hardware prerequisite (wiring note, not code): HC-SR04 ECHO outputs 5 V but
 * RP2040 GPIO is 3.3 V tolerant only - a resistor voltage divider (or level
 * shifter) on the ECHO line is MANDATORY before GP17.
 * ========================================================================== */

/* ----- Pin map ----- */
#define US_TRIG_PIN        16u   /* GP16, GPIO output */
#define US_ECHO_PIN        17u   /* GP17, GPIO input (via 5V->3.3V divider!) */
#define SRV_PWM_PIN        12u   /* GP12, PWM, dedicated servo slice */

/* ----- HC-SR04 timing / conversion ----- */
#define US_TRIG_PULSE_US   10u    /* >= 10 us TTL high to start ranging */
#define US_CYCLE_MS        60u    /* >= 60 ms between triggers (hardware limit) */
#define US_ECHO_TIMEOUT_US 25000u /* ~430 cm round-trip; beyond -> NO_ECHO */
#define US_CM_DIVISOR      58u    /* distance_cm = echo_high_us / 58 */

/* ----- Servo (50 Hz) ----- */
#define SRV_PWM_HZ         50u
#define SRV_SETTLE_MS      200u   /* within the 150..300 ms window */
#define SRV_MIN_DEG        30u
#define SRV_MAX_DEG        150u

/* ----- Scan behavior ----- */
#define COARSE_ANGLES        {30u, 60u, 90u, 120u, 150u}
#define READINGS_PER_ANGLE   5u   /* multiple reads; filtered to one result */
#define SAFE_LIMIT_CM        30u  /* proximity threshold to escalate */
#define FAULT_NO_ECHO_STREAK 3u   /* consecutive timeouts before latching fault */

#endif /* OBSTACLE_INTEGRATION_H */
