/*
 * Ultrasonic Obstacle Profiling (Buddy 5 / Subsystem 5) - pure logic.
 *
 * PURE, host-testable logic functions only: no Pico SDK / hardware calls, so
 * these can be exercised off-device by the TEST_CHECK harness to verify the
 * Correctness Properties. Multiple pure-logic tasks share this translation
 * unit; each APPENDS its own definitions. Do not clobber sibling functions.
 *
 * Conventions: Barr-C style - snake_case, header guards in the .h.
 */

#include "obstacle_logic.h"

/* ==========================================================================
 * Task 2.4: obs_pack_telemetry / obs_unpack_telemetry
 * --------------------------------------------------------------------------
 * Pack the obstacle profile into the 32-bit telemetry data word, one field
 * per byte lane (no truncation, no cross-field overlap):
 *   [31:24] flags | [23:16] center_angle_deg |
 *   [15: 8] closest_distance_cm | [7:0] approx_width_cm
 * _Requirements: 5.1, 5.2_
 * ========================================================================== */

uint32_t obs_pack_telemetry(const obstacle_profile_t *p, uint8_t flags)
{
    return ((uint32_t)flags                  << 24)
         | ((uint32_t)p->center_angle_deg     << 16)
         | ((uint32_t)p->closest_distance_cm  <<  8)
         |  (uint32_t)p->approx_width_cm;
}

uint8_t obs_unpack_telemetry_flags(uint32_t packed)
{
    return (uint8_t)((packed >> 24) & 0xFFu);
}

uint8_t obs_unpack_telemetry_center_angle_deg(uint32_t packed)
{
    return (uint8_t)((packed >> 16) & 0xFFu);
}

uint8_t obs_unpack_telemetry_closest_distance_cm(uint32_t packed)
{
    return (uint8_t)((packed >> 8) & 0xFFu);
}

uint8_t obs_unpack_telemetry_approx_width_cm(uint32_t packed)
{
    return (uint8_t)(packed & 0xFFu);
}

/* ==========================================================================
 * Task 4.1: plan_bypass
 * --------------------------------------------------------------------------
 * Pure avoidance-planning decision. Returns MOTOR_CMD_BYPASS_LEFT iff the
 * estimated left clearance is strictly greater than the estimated right
 * clearance; otherwise MOTOR_CMD_BYPASS_RIGHT. The "otherwise" branch makes
 * ties (left <= right) resolve deterministically to the right, so the
 * decision is total and repeatable.
 * _Requirements: 3.1, 3.2_
 * ========================================================================== */

motor_cmd_t plan_bypass(const obstacle_profile_t *p)
{
    motor_cmd_t cmd = MOTOR_CMD_BYPASS_RIGHT;

    if (p->left_clearance_cm > p->right_clearance_cm) {
        cmd = MOTOR_CMD_BYPASS_LEFT;
    }

    return cmd;
}

/* ==========================================================================
 * Task 3.1: filter_samples
 * --------------------------------------------------------------------------
 * Modal/median reducer over the raw samples taken at one servo angle. Pure and
 * host-testable (injected array), kept separate from the sampling loop.
 *   - Discard every sample equal to OBS_DISTANCE_NO_ECHO (255).
 *   - All-NO_ECHO (or n == 0) -> OBS_DISTANCE_NO_ECHO.
 *   - Return the mode when any valid value occurs >= 2 times; frequency ties
 *     resolve to the smallest such value (deterministic, prefers the nearer
 *     reading).
 *   - Otherwise return the median of the valid samples (lower-middle element
 *     for an even count, so the result is always an actually-measured value).
 * _Requirements: 1.6, 1.7, 2.2, 2.3, 2.4_
 * ========================================================================== */

uint8_t filter_samples(const uint8_t *samples, uint8_t n)
{
    uint8_t  valid[256];      /* holds the non-NO_ECHO samples */
    uint16_t valid_count = 0u;
    uint16_t i;
    uint16_t j;

    if ((samples == 0) || (n == 0u)) {
        return OBS_DISTANCE_NO_ECHO;
    }

    /* 1. Discard NO_ECHO samples, keep the valid ones. */
    for (i = 0u; i < (uint16_t)n; i++) {
        if (samples[i] != OBS_DISTANCE_NO_ECHO) {
            valid[valid_count] = samples[i];
            valid_count++;
        }
    }

    /* 2. No valid samples -> NO_ECHO for this angle. (Reqs 1.7, 2.4) */
    if (valid_count == 0u) {
        return OBS_DISTANCE_NO_ECHO;
    }

    /* Insertion sort ascending: makes equal values adjacent (cheap mode scan)
     * and gives a direct median index on the small fixed array. */
    for (i = 1u; i < valid_count; i++) {
        uint8_t  key = valid[i];
        uint16_t k   = i;

        while ((k > 0u) && (valid[k - 1u] > key)) {
            valid[k] = valid[k - 1u];
            k--;
        }
        valid[k] = key;
    }

    /* 3. Mode: longest run of equal values. Because the array is sorted and a
     * run only replaces the best when STRICTLY longer, the smallest value wins
     * a frequency tie. (Reqs 1.6, 2.2) */
    {
        uint8_t  best_value = valid[0];
        uint16_t best_count = 1u;

        i = 0u;
        while (i < valid_count) {
            uint8_t  run_value = valid[i];
            uint16_t run_count = 0u;

            for (j = i; (j < valid_count) && (valid[j] == run_value); j++) {
                run_count++;
            }

            if (run_count > best_count) {
                best_count = run_count;
                best_value = run_value;
            }

            i = j; /* advance to the next distinct value */
        }

        if (best_count >= 2u) {
            return best_value;
        }
    }

    /* 4. All valid values distinct -> median (lower-middle for even count,
     * never an averaged/synthetic value). (Req 2.3) */
    return valid[(valid_count - 1u) / 2u];
}

/* ==========================================================================
 * Task 2.1: us_echo_to_cm
 * --------------------------------------------------------------------------
 * Convert an HC-SR04 ECHO high-time (microseconds) to a distance in cm.
 *
 *   echo_us == 0            -> OBS_DISTANCE_NO_ECHO (255): a timeout.
 *   echo_us  > 0           -> echo_us / US_CM_DIVISOR (58), then
 *                               floored up to OBS_DISTANCE_MIN_CM (2) if below,
 *                               clamped to OBS_DISTANCE_MAX_CM (254) if above.
 *
 * Integer division is monotonic non-decreasing, so larger echo times never
 * yield a smaller distance. The output is 255 iff echo_us == 0; every non-zero
 * input lands in {2..254}, so the valid ceiling (254) stays distinct from the
 * NO_ECHO sentinel (255).
 * _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 2.1_
 * ========================================================================== */

/* Minimum reportable valid distance (cm). Conversions below this are floored
 * up to it so a tiny-but-nonzero echo is never confused with a timeout. */
#define OBS_DISTANCE_MIN_CM  ((uint8_t)2u)

uint8_t us_echo_to_cm(uint32_t echo_us)
{
    uint32_t cm;

    if (0u == echo_us) {
        return OBS_DISTANCE_NO_ECHO; /* timeout -> sentinel, never a valid reading */
    }

    cm = echo_us / US_CM_DIVISOR; /* monotonic non-decreasing integer division */

    if (cm < (uint32_t)OBS_DISTANCE_MIN_CM) {
        cm = (uint32_t)OBS_DISTANCE_MIN_CM; /* floor below-spec readings to 2 cm */
    } else if (cm > (uint32_t)OBS_DISTANCE_MAX_CM) {
        cm = (uint32_t)OBS_DISTANCE_MAX_CM; /* clamp to 254; stays below 255 */
    } else {
        /* in-range: use as computed */
    }

    return (uint8_t)cm;
}

/* ==========================================================================
 * Task 4.3: estimate_profile
 * --------------------------------------------------------------------------
 * Reduce a scanned region (injected array of filtered per-angle readings) to a
 * single obstacle_profile_t ESTIMATE. Pure and host-testable: it reads only the
 * caller-provided array and never touches the servo / HC-SR04.
 *
 * Only readings whose angle lies within the inclusive [lo, hi] scan window are
 * considered; every NO_ECHO reading is ignored for all statistics.
 *   - closest_distance_cm : min valid distance over the window.            (2.10)
 *   - center_angle_deg    : angle of that closest valid reading; on a tie the
 *                           first (smallest-angle) occurrence wins so the result
 *                           is deterministic.                              (2.11)
 *   - left_clearance_cm   : min valid distance over angles > center.       (2.12)
 *   - right_clearance_cm  : min valid distance over angles < center.       (2.13)
 *                           An empty side defaults to OBS_DISTANCE_MAX_CM
 *                           (treated as fully open) so planning still has a
 *                           total ordering to compare.
 *   - approx_width_cm     : ESTIMATE only (2.9) - the contiguous angular span
 *                           of near-returns around center converted to a linear
 *                           width at the closest distance, clamped to 0..254.
 *   - present             : closest_distance_cm <= SAFE_LIMIT_CM.
 *
 * A window with no valid reading is reported not-present with defaulted fields.
 * _Requirements: 2.9, 2.10, 2.11, 2.12, 2.13_
 * ========================================================================== */

/* A return counts as part of the same obstacle face as the closest point when
 * it is within this many cm of the closest distance. Used only to bound the
 * contiguous angular span that feeds the width ESTIMATE. */
#define OBS_WIDTH_NEAR_TOL_CM  ((uint8_t)10u)

/* Fixed-point approximation of pi/180 scaled by 10000 (0.017453 -> 175).
 * width ~= closest * span_deg * (pi/180); with small ~15 deg beam steps the
 * linear small-angle approximation is honest enough for an ESTIMATE and avoids
 * pulling float / libm into the pure logic. */
#define OBS_DEG_TO_LEN_NUM  175u
#define OBS_DEG_TO_LEN_DEN  10000u

static bool obs_reading_in_window(const angle_reading_t *r, uint8_t lo,
                                  uint8_t hi)
{
    return (r->angle_deg >= lo)
        && (r->angle_deg <= hi)
        && (r->distance_cm != OBS_DISTANCE_NO_ECHO);
}

obstacle_profile_t estimate_profile(const angle_reading_t *readings,
                                    uint8_t count, uint8_t lo, uint8_t hi)
{
    obstacle_profile_t profile;
    uint16_t           i;
    bool               have_any     = false;
    uint8_t            closest       = OBS_DISTANCE_MAX_CM;
    uint8_t            center        = lo;
    uint8_t            left_clear    = OBS_DISTANCE_MAX_CM; /* angles > center */
    uint8_t            right_clear   = OBS_DISTANCE_MAX_CM; /* angles < center */
    uint8_t            span_lo_deg;
    uint8_t            span_hi_deg;
    uint16_t           span_deg;
    uint32_t           width;

    /* Defaults for an empty / all-NO_ECHO window: nothing confirmed. */
    profile.present             = false;
    profile.closest_distance_cm = OBS_DISTANCE_MAX_CM;
    profile.center_angle_deg    = lo;
    profile.approx_width_cm      = 0u;
    profile.left_clearance_cm    = OBS_DISTANCE_MAX_CM;
    profile.right_clearance_cm   = OBS_DISTANCE_MAX_CM;

    if ((readings == 0) || (count == 0u)) {
        return profile;
    }

    /* Pass 1: closest valid distance and its angle (smallest-angle tie-break).
     * A single forward scan with a strict '<' keeps the first occurrence, so a
     * frequency/distance tie resolves to the smaller angle deterministically. */
    for (i = 0u; i < (uint16_t)count; i++) {
        if (obs_reading_in_window(&readings[i], lo, hi)) {
            if ((!have_any) || (readings[i].distance_cm < closest)) {
                closest = readings[i].distance_cm;
                center  = readings[i].angle_deg;
            }
            have_any = true;
        }
    }

    if (!have_any) {
        return profile; /* window held no valid return -> keep the defaults */
    }

    /* Pass 2: side clearances as the min valid distance strictly to each side
     * of the center angle (2.12 left = angles > center, 2.13 right = angles <
     * center). Sides with no valid reading stay fully open (MAX_CM). */
    for (i = 0u; i < (uint16_t)count; i++) {
        if (obs_reading_in_window(&readings[i], lo, hi)) {
            if (readings[i].angle_deg > center) {
                if (readings[i].distance_cm < left_clear) {
                    left_clear = readings[i].distance_cm;
                }
            } else if (readings[i].angle_deg < center) {
                if (readings[i].distance_cm < right_clear) {
                    right_clear = readings[i].distance_cm;
                }
            } else {
                /* the center reading itself: not a side clearance */
            }
        }
    }

    /* Pass 3: contiguous angular span of near-returns around center. Walk out
     * from center in angle order while readings stay within NEAR_TOL of the
     * closest distance; the min/max of that run bounds the span. */
    span_lo_deg = center;
    span_hi_deg = center;
    for (i = 0u; i < (uint16_t)count; i++) {
        if (obs_reading_in_window(&readings[i], lo, hi)) {
            uint8_t d = readings[i].distance_cm;
            uint8_t delta = (d > closest) ? (uint8_t)(d - closest)
                                          : (uint8_t)(closest - d);
            if (delta <= OBS_WIDTH_NEAR_TOL_CM) {
                if (readings[i].angle_deg < span_lo_deg) {
                    span_lo_deg = readings[i].angle_deg;
                }
                if (readings[i].angle_deg > span_hi_deg) {
                    span_hi_deg = readings[i].angle_deg;
                }
            }
        }
    }

    span_deg = (uint16_t)(span_hi_deg - span_lo_deg);

    /* ESTIMATE only: linear width ~= closest * span_deg * (pi/180). Integer
     * fixed-point keeps the pure logic free of float/libm, then clamp to the
     * 0..254 uint8_t distance range. */
    width = ((uint32_t)closest * (uint32_t)span_deg * OBS_DEG_TO_LEN_NUM)
            / OBS_DEG_TO_LEN_DEN;
    if (width > (uint32_t)OBS_DISTANCE_MAX_CM) {
        width = (uint32_t)OBS_DISTANCE_MAX_CM;
    }

    profile.present             = (closest <= (uint8_t)SAFE_LIMIT_CM);
    profile.closest_distance_cm = closest;
    profile.center_angle_deg    = center;
    profile.approx_width_cm      = (uint8_t)width;
    profile.left_clearance_cm    = left_clear;
    profile.right_clearance_cm   = right_clear;

    return profile;
}

/* ==========================================================================
 * Task 7.1: servo_clamp_angle (pure helper for the servo driver)
 * --------------------------------------------------------------------------
 * Clamp a commanded servo angle into [SRV_MIN_DEG, SRV_MAX_DEG] and flag when
 * the request fell outside that range. Pure: no PWM / hardware calls, so the
 * "servo angle bounded" property is host-testable. The servo driver uses the
 * returned angle to drive the PWM and latches FAULT_SERVO when *out_of_range
 * comes back true, keeping the fault side effect out of this pure function.
 * _Requirements: 2.5, 3.11_
 * ========================================================================== */

uint8_t servo_clamp_angle(uint8_t angle_deg, bool *out_of_range)
{
    uint8_t clamped = angle_deg;
    bool    outside = false;

    if (angle_deg < (uint8_t)SRV_MIN_DEG) {
        clamped = (uint8_t)SRV_MIN_DEG; /* below range -> nearest bound */
        outside = true;
    } else if (angle_deg > (uint8_t)SRV_MAX_DEG) {
        clamped = (uint8_t)SRV_MAX_DEG; /* above range -> nearest bound */
        outside = true;
    } else {
        /* in range: use as commanded */
    }

    if (out_of_range != 0) {
        *out_of_range = outside;
    }

    return clamped;
}

/* ==========================================================================
 * Task 13.3: obs_should_escalate (pure IDLE_MONITORING transition guard)
 * --------------------------------------------------------------------------
 * The pure, host-testable form of obstacle_task's IDLE_MONITORING escalation
 * decision (obstacle_avoidance.c). Returns true IFF the terrain is clear of a
 * hump AND the idle reading is a real echo strictly inside the safe limit:
 *
 *   hump == HUMP_NONE                      (req 1.11: ignore while hump active)
 *   && distance_cm != OBS_DISTANCE_NO_ECHO (a timeout is not a real return)
 *   && distance_cm <  SAFE_LIMIT_CM        (req 1.8: escalate only inside limit)
 *
 * No hardware / Buddy 4 calls - the caller passes the live terrain_hump_state()
 * and the latest filtered idle reading, keeping this decision host-testable.
 * _Requirements: 1.8, 1.11_
 * ========================================================================== */

bool obs_should_escalate(uint8_t distance_cm, hump_state_t hump)
{
    return (HUMP_NONE == hump)
        && (OBS_DISTANCE_NO_ECHO != distance_cm)
        && (distance_cm < (uint8_t)SAFE_LIMIT_CM);
}
