#ifndef OBSTACLE_LOGIC_H
#define OBSTACLE_LOGIC_H

/*
 * Ultrasonic Obstacle Profiling (Buddy 5 / Subsystem 5) - pure logic header.
 *
 * This header declares the subsystem's PURE, host-testable logic functions:
 * functions that depend on no Pico SDK / hardware headers and operate only on
 * their inputs, so the Correctness Properties can be checked off-device with
 * the repo's TEST_CHECK harness.
 *
 * Multiple pure-logic tasks (us_echo_to_cm, obs_pack_motor_cmd / unpack,
 * obs_pack_telemetry / unpack, filter_samples, plan_bypass, estimate_profile)
 * share this single translation unit (obstacle_logic.c). Each task APPENDS its
 * own declarations here and definitions in the .c; do not clobber siblings.
 *
 * Conventions: Barr-C style - snake_case functions, #define macros, header
 * guards. No hardware headers.
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_avoidance.h"     /* OBS_DISTANCE_NO_ECHO / _MAX_CM, FAULT_* */
#include "obstacle_integration.h"   /* motor_cmd_t and friends */

/* --------------------------------------------------------------------------
 * Obstacle profile (FR7 output - all fields are ESTIMATES)
 * --------------------------------------------------------------------------
 * One HC-SR04 with a ~15 deg beam cannot recover true geometry; every field
 * here is an estimate. Defined in the shared pure-logic header because the
 * packing and planning logic operate on it without touching hardware.
 */
typedef struct {
    bool     present;             /* obstacle confirmed? */
    uint8_t  closest_distance_cm; /* min valid distance across the region */
    uint8_t  center_angle_deg;    /* servo angle of the closest return */
    uint8_t  approx_width_cm;     /* ESTIMATE, bounded by ~15 deg beam width */
    uint8_t  left_clearance_cm;   /* ESTIMATE of open space to the left */
    uint8_t  right_clearance_cm;  /* ESTIMATE of open space to the right */
} obstacle_profile_t;

/* --------------------------------------------------------------------------
 * Single filtered reading at one servo angle - used by estimate_profile (4.3)
 * --------------------------------------------------------------------------
 * One filtered distance measurement taken at a known servo angle. A region
 * (coarse or fine scan) is an injected array of these, so the profiling logic
 * stays pure and host-testable (no servo / HC-SR04 calls).
 *   angle_deg   : the servo angle the reading was taken at (30..150 in use)
 *   distance_cm : 2..254 for a valid return, or OBS_DISTANCE_NO_ECHO (255)
 */
typedef struct {
    uint8_t angle_deg;    /* 30..150 */
    uint8_t distance_cm;  /* 2..254 valid, or OBS_DISTANCE_NO_ECHO */
} angle_reading_t;

/* --------------------------------------------------------------------------
 * uint32_t telemetry packing (obstacle message) - task 2.4
 * --------------------------------------------------------------------------
 * The 32-bit telemetry data field carries flags/angle/distance/width in four
 * byte lanes with no truncation and no cross-field overlap:
 *
 *   bits [31:24] flags              (e.g. bit24 = FAULT_NO_ECHO)
 *   bits [23:16] center_angle_deg   (0..180)
 *   bits [15: 8] closest_distance_cm (0..254, 255 = NO_ECHO)
 *   bits [ 7: 0] approx_width_cm
 */

/* Pack a profile plus a flags byte into the 32-bit telemetry data word. */
uint32_t obs_pack_telemetry(const obstacle_profile_t *p, uint8_t flags);

/* Inverse helpers: extract each byte lane from a packed telemetry word. */
uint8_t obs_unpack_telemetry_flags(uint32_t packed);
uint8_t obs_unpack_telemetry_center_angle_deg(uint32_t packed);
uint8_t obs_unpack_telemetry_closest_distance_cm(uint32_t packed);
uint8_t obs_unpack_telemetry_approx_width_cm(uint32_t packed);

/* --------------------------------------------------------------------------
 * 16-bit motor command packing - task 2.3
 * --------------------------------------------------------------------------
 * A posted motor command is a single 16-bit value laid out as:
 *
 *   bit:  15 14 | 13 ....................... 0
 *         cmd   | payload (14 bits, 0..16383)
 *
 * [15:14] = command  (motor_cmd_t, 0..3)
 * [13:0]  = payload   (speed or distance_ticks, 0..16383)
 *
 * Packing masks each field to its lane so an over-wide input can never bleed
 * into the neighbouring field; the inverse unpack helpers recover each field
 * exactly, so pack followed by unpack is a lossless round-trip for in-range
 * inputs (Correctness Property 7). Provided as static inline per design.md so
 * they are zero-cost and usable from both device and host code.
 * _Requirements: 3.3_
 */

/* Field shifts / masks for the 16-bit motor command word. */
#define OBS_MOTOR_CMD_SHIFT       14u
#define OBS_MOTOR_CMD_MASK        0x3u      /* 2-bit command field  */
#define OBS_MOTOR_PAYLOAD_MASK    0x3FFFu   /* 14-bit payload field */

/* Pack a command + 14-bit payload into the 16-bit on-the-wire value.
 * Each field is masked to its lane so no field can corrupt the other. */
static inline uint16_t obs_pack_motor_cmd(motor_cmd_t cmd, uint16_t payload14)
{
    return (uint16_t)(((uint16_t)((uint16_t)cmd & OBS_MOTOR_CMD_MASK)
                       << OBS_MOTOR_CMD_SHIFT)
                      | (uint16_t)(payload14 & OBS_MOTOR_PAYLOAD_MASK));
}

/* Unpack the command field (bits [15:14]); result is 0..3. */
static inline motor_cmd_t obs_unpack_motor_cmd(uint16_t packed)
{
    return (motor_cmd_t)((packed >> OBS_MOTOR_CMD_SHIFT) & OBS_MOTOR_CMD_MASK);
}

/* Unpack the payload field (bits [13:0]); result is 0..16383. */
static inline uint16_t obs_unpack_motor_payload(uint16_t packed)
{
    return (uint16_t)(packed & OBS_MOTOR_PAYLOAD_MASK);
}

/* --------------------------------------------------------------------------
 * Avoidance planning (left/right decision) - task 4.1
 * --------------------------------------------------------------------------
 * Choose the bypass side with the greater estimated clearance. Returns
 * MOTOR_CMD_BYPASS_LEFT when left_clearance_cm is strictly greater than
 * right_clearance_cm, otherwise MOTOR_CMD_BYPASS_RIGHT (deterministic
 * tie-break on equality, so the decision is total and repeatable).
 * _Requirements: 3.1, 3.2_
 */
motor_cmd_t plan_bypass(const obstacle_profile_t *p);

/* --------------------------------------------------------------------------
 * Modal/median sample filter - task 3.1
 * --------------------------------------------------------------------------
 * Reduce n raw per-angle samples to a single filtered distance:
 *   1. Every sample equal to OBS_DISTANCE_NO_ECHO (255) is discarded.
 *   2. If no valid samples remain (all were NO_ECHO, or n == 0), the result is
 *      OBS_DISTANCE_NO_ECHO.
 *   3. Otherwise, if any valid value occurs 2 or more times, the mode is
 *      returned (the most frequent value; on a tie in frequency the smallest
 *      such value wins - deterministic and collision-conservative, preferring
 *      the nearer reading).
 *   4. Otherwise (all valid values distinct) the median of the valid samples
 *      is returned (lower-middle element for an even count, so the result is
 *      always an actually-measured integer distance - no averaging).
 *
 * Pure: reads only the caller-provided array, so it is host-testable with
 * injected arrays independent of the sampling loop. samples may be NULL only
 * when n == 0.
 * _Requirements: 1.6, 1.7, 2.2, 2.3, 2.4_
 */
uint8_t filter_samples(const uint8_t *samples, uint8_t n);

/* --------------------------------------------------------------------------
 * Echo-time to centimetres conversion - task 2.1
 * --------------------------------------------------------------------------
 * Convert a measured HC-SR04 ECHO high-time in microseconds to a distance in
 * centimetres:
 *   - echo_us == 0 (a timeout) returns the OBS_DISTANCE_NO_ECHO sentinel (255).
 *   - Otherwise the value is integer-divided by US_CM_DIVISOR (58), then floored
 *     up to the 2 cm minimum when below it and clamped to OBS_DISTANCE_MAX_CM
 *     (254) when above it.
 *
 * Integer division is monotonic non-decreasing, so a larger echo_us never
 * yields a smaller distance. The result is 255 if and only if echo_us == 0;
 * every non-zero input maps into {2..254}, keeping a valid far reading (254)
 * always distinct from the NO_ECHO sentinel (255).
 *
 * Pure: depends only on echo_us, so it is host-testable off-device.
 * _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 2.1_
 */
uint8_t us_echo_to_cm(uint32_t echo_us);

/* --------------------------------------------------------------------------
 * Obstacle profile estimation - task 4.3
 * --------------------------------------------------------------------------
 * Reduce a scanned region (an injected array of filtered per-angle readings)
 * to a single obstacle_profile_t estimate. Only readings whose angle_deg lies
 * within the inclusive [lo, hi] scan window are considered; NO_ECHO readings
 * are ignored for every statistic. All profile fields are ESTIMATES (a single
 * ~15 deg beam cannot recover true geometry).
 *
 *   closest_distance_cm : minimum valid distance over the window.
 *   center_angle_deg    : angle of that closest valid reading (on a tie the
 *                          first/smallest-angle occurrence wins -> deterministic).
 *   left_clearance_cm   : minimum valid distance over angles > center; when no
 *                          valid reading exists on that side, OBS_DISTANCE_MAX_CM
 *                          (treated as fully open).
 *   right_clearance_cm  : minimum valid distance over angles < center; same
 *                          open-side fallback as above.
 *   approx_width_cm     : ESTIMATE from the contiguous angular span of near
 *                          returns around center, converted to a linear width
 *                          at the closest distance and clamped to 0..254.
 *   present             : true iff closest_distance_cm <= SAFE_LIMIT_CM.
 *
 * When the window contains no valid reading at all, the profile is reported as
 * not present with closest/width/clearances defaulted so downstream planning
 * sees "nothing confirmed".
 *
 * Pure: reads only the caller-provided array; readings may be NULL only when
 * count == 0.
 * _Requirements: 2.9, 2.10, 2.11, 2.12, 2.13_
 */
obstacle_profile_t estimate_profile(const angle_reading_t *readings,
                                    uint8_t count, uint8_t lo, uint8_t hi);

/* --------------------------------------------------------------------------
 * Servo angle clamp - task 7.1 (pure helper for the servo driver)
 * --------------------------------------------------------------------------
 * Clamp a commanded servo angle to the mechanical/scan range
 * [SRV_MIN_DEG, SRV_MAX_DEG] and report whether the request was out of range.
 *
 * Kept PURE (no Pico SDK / PWM calls) so Correctness Property 5 (servo angle
 * bounded) can be checked off-device by the TEST_CHECK harness (task 7.2). The
 * hardware servo driver (servo.c) calls this, drives the PWM with the returned
 * in-range angle, and sets FAULT_SERVO when this reports out-of-range - so the
 * fault side effect stays out of the pure logic.
 *
 *   return value             : the clamped angle, always in [30, 150].
 *   *out_of_range (optional) : set true when angle_deg was outside the range
 *                              (so the caller latches FAULT_SERVO), else false.
 *                              May be NULL if the caller does not need it.
 * _Requirements: 2.5, 3.11_
 */
uint8_t servo_clamp_angle(uint8_t angle_deg, bool *out_of_range);

/* --------------------------------------------------------------------------
 * IDLE_MONITORING escalation guard - task 13.3 (pure predicate)
 * --------------------------------------------------------------------------
 * Decide, from a single idle-monitoring reading, whether the obstacle_task
 * should escalate from IDLE_MONITORING to COARSE_SCAN. This is the pure,
 * host-testable form of the inline condition in obstacle_task's
 * IDLE_MONITORING arm (obstacle_avoidance.c), so the hump-suppression
 * transition guard can be checked off-device with the TEST_CHECK harness.
 *
 * Returns true IFF ALL of the following hold:
 *   - hump == HUMP_NONE               : an active hump/incline reads like a wall
 *                                       to a single HC-SR04, so while a hump is
 *                                       active any proximity trigger is IGNORED
 *                                       (req 1.11) - no escalation.
 *   - distance_cm != OBS_DISTANCE_NO_ECHO : a timeout is not a real return, so a
 *                                       NO_ECHO reading never escalates.
 *   - distance_cm < SAFE_LIMIT_CM     : only a real echo strictly inside the
 *                                       safe limit escalates to the coarse scan
 *                                       (req 1.8).
 *
 * Pure: depends only on its two arguments (no hardware, no Buddy 4 call), so it
 * is host-testable. The state machine passes the live terrain_hump_state() and
 * the latest filtered idle reading; the hump-first ordering of the real arm is
 * preserved separately so the "ignore while hump active" idle-delay path is
 * unchanged.
 * _Requirements: 1.8, 1.11_
 */
bool obs_should_escalate(uint8_t distance_cm, hump_state_t hump);

#endif /* OBSTACLE_LOGIC_H */
