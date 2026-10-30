/*
 * Ultrasonic Obstacle Profiling (Buddy 5 / Subsystem 5) - host-testable tests.
 *
 * CANONICAL test harness (task 5) for the subsystem's PURE logic. It holds the
 * five pure-logic test functions and a single runner main() that invokes them:
 *   test_us_echo_to_cm_props   (2.2 - Properties 1/2/3, us_echo_to_cm)
 *   test_packing_roundtrips    (2.5 - Property 7, obs_pack_motor_cmd + telemetry)
 *   test_filter_samples_props  (3.2 - Property 4, filter_samples)
 *   test_plan_bypass_props     (4.2 - Property 6, plan_bypass)
 *   test_estimate_profile_unit (4.4 - estimate_profile unit tests)
 *
 * The tests use the repo's `TEST_CHECK(id, cond)` convention: each check prints
 * one `[TEST] <id> PASS|FAIL` line (see testing/README.md). Every S5-Uxx ID is
 * unique across the file and registered in testing/subsystem_5/TEST_LOG.md.
 * TEST_CHECK and the tiny pass/fail counters are defined with `#ifndef` guards
 * (single canonical definition, no duplicate harness).
 *
 * Host build (off-device, no hardware): the pure logic reaches FreeRTOS headers
 * only transitively through obstacle_avoidance.h, so a `host_stubs/` shim
 * directory supplies FreeRTOS.h / queue.h / task.h for the host compiler. The
 * cycle-floor test (task 9.2) additionally compiles obstacle_avoidance.c so it
 * measures the REAL aim_and_read() trigger spacing; that TU owns a placeholder
 * demo main(), so it is compiled with -Dmain=... to free this file's runner
 * main(), and its hardware symbols (servo_set_angle / servo_fault /
 * us_measure_echo_us) are provided as link-only stubs in this file. Example:
 *
 *   gcc -std=c11 -I app/subsystem_5 -I app/subsystem_5/host_stubs \
 *       -Dmain=obstacle_avoidance_demo_main -c \
 *       app/subsystem_5/obstacle_avoidance.c -o build_host/obstacle_avoidance.o
 *   gcc -std=c11 -I app/subsystem_5 -I app/subsystem_5/host_stubs -c \
 *       app/subsystem_5/obstacle_logic.c -o build_host/obstacle_logic.o
 *   gcc -std=c11 -I app/subsystem_5 -I app/subsystem_5/host_stubs -c \
 *       app/subsystem_5/obstacle_test.c -o build_host/obstacle_test.o
 *   gcc build_host/obstacle_avoidance.o build_host/obstacle_logic.o \
 *       build_host/obstacle_test.o -o build_host/obstacle_test &&
 *   ./build_host/obstacle_test
 *
 * Conventions: Barr-C style - snake_case, header guards, #define macros.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "obstacle_logic.h"
#include "obstacle_scan.h"     /* aim_and_read + timing seams (task 9.2) */

/* --------------------------------------------------------------------------
 * Minimal TEST_CHECK harness (guarded so task 5's canonical macro wins if it
 * is included/defined first). One line per check, plus running pass/fail totals
 * so a host run can exit non-zero when anything failed.
 * -------------------------------------------------------------------------- */
#ifndef TEST_CHECK
static unsigned int g_test_pass_count = 0u;
static unsigned int g_test_fail_count = 0u;

#define TEST_CHECK(id, cond)                                      \
    do {                                                          \
        if (cond) {                                               \
            g_test_pass_count++;                                  \
            printf("[TEST] %s PASS\n", (id));                     \
        } else {                                                  \
            g_test_fail_count++;                                  \
            printf("[TEST] %s FAIL  (%s:%d: %s)\n",               \
                   (id), __FILE__, __LINE__, #cond);              \
        }                                                         \
    } while (0)
#endif /* TEST_CHECK */

/* ==========================================================================
 * Shared deterministic PRNG (xorshift32).
 * --------------------------------------------------------------------------
 * A single self-contained, reproducible generator shared by every property
 * test in this file (host-only, no external PBT library on device). Seed is
 * fixed for repeatability across runs and hosts.
 * ========================================================================== */
#ifndef OBS_PRNG_DEFINED
#define OBS_PRNG_DEFINED
static uint32_t obs_prng_state = 0x1234ABCDu ^ 0xC0FFEEu; /* fixed seed */

static uint32_t obs_prng_next(void)
{
    uint32_t x = obs_prng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    obs_prng_state = x;
    return x;
}

/* Uniform integer in [lo, hi] inclusive (lo <= hi). */
static uint32_t obs_prng_range(uint32_t lo, uint32_t hi)
{
    return lo + (obs_prng_next() % ((hi - lo) + 1u));
}
#endif /* OBS_PRNG_DEFINED */

/* ==========================================================================
 * Task 2.2 - Properties 1/2/3 for us_echo_to_cm.
 * --------------------------------------------------------------------------
 * us_echo_to_cm(echo_us) converts a measured HC-SR04 ECHO high-time in
 * microseconds into a distance in centimetres. Three Correctness Properties
 * must hold:
 *
 *   Property 1 (Distance monotonicity): for non-zero a < b,
 *       us_echo_to_cm(a) <= us_echo_to_cm(b). More echo time never maps to a
 *       nearer distance.
 *   Property 2 (Clamp ceiling): for any non-zero echo_us, the result is <= 254
 *       (OBS_DISTANCE_MAX_CM) - a valid reading never exceeds the clamp.
 *   Property 3 (NO_ECHO distinguishable): result == 255 (OBS_DISTANCE_NO_ECHO)
 *       if and only if echo_us == 0, so a timeout is always distinguishable
 *       from a genuine far reading (254).
 *
 * This is a PROPERTY test: the generators sweep the echo-time input space with
 * the shared deterministic PRNG across many randomized non-zero values (and the
 * echo_us == 0 edge is tested exhaustively for Property 3). The generators are
 * smart - they draw non-zero values across the full meaningful range (from a
 * 1 us sub-floor echo through the 25000 us timeout window and well beyond, up
 * to UINT32_MAX) so the floor, the in-range band, and the ceiling clamp are all
 * exercised. Monotonicity is checked on ordered random pairs.
 *
 * **Validates: Requirements 1.1, 1.2, 1.3, 1.5, 2.1**
 * ========================================================================== */

#define OBS_P123_ITERATIONS  5000u  /* randomized cases to exercise */

void test_us_echo_to_cm_props(void)
{
    uint32_t iter;
    bool     monotonic_always     = true; /* Property 1 */
    bool     clamp_ceiling_always = true; /* Property 2 */
    bool     sentinel_iff_zero    = true; /* Property 3 */

    for (iter = 0u; iter < OBS_P123_ITERATIONS; iter++) {
        uint32_t a;
        uint32_t b;
        uint32_t lo;
        uint32_t hi;
        uint8_t  da;
        uint8_t  db;

        /* Two independent non-zero echo times spanning the whole range: the
         * sub-floor band (1..115 -> clamps up to 2 cm), the linear band, and
         * the saturating band above 254*58 us (-> 254 cm). */
        a = obs_prng_range(1u, 0xFFFFFFFFu);
        b = obs_prng_range(1u, 0xFFFFFFFFu);

        /* Property 2: every non-zero input is clamped to <= 254. */
        da = us_echo_to_cm(a);
        db = us_echo_to_cm(b);
        if ((da > OBS_DISTANCE_MAX_CM) || (db > OBS_DISTANCE_MAX_CM)) {
            clamp_ceiling_always = false;
        }

        /* Property 3 (non-zero direction): a non-zero input is NEVER 255. */
        if ((da == OBS_DISTANCE_NO_ECHO) || (db == OBS_DISTANCE_NO_ECHO)) {
            sentinel_iff_zero = false;
        }

        /* Property 1: order the pair then require non-decreasing distance. */
        if (a <= b) {
            lo = a;
            hi = b;
        } else {
            lo = b;
            hi = a;
        }
        if (us_echo_to_cm(lo) > us_echo_to_cm(hi)) {
            monotonic_always = false;
        }
    }

    /* Property 3 (zero direction), exhaustive on the single edge value: a zero
     * echo time maps to the sentinel and nothing else does. */
    if (us_echo_to_cm(0u) != OBS_DISTANCE_NO_ECHO) {
        sentinel_iff_zero = false;
    }

    /* S5-U10: Property 1 - distance is monotonic non-decreasing in echo_us. */
    TEST_CHECK("S5-U10", monotonic_always);
    /* S5-U11: Property 2 - every non-zero echo clamps to <= 254 cm. */
    TEST_CHECK("S5-U11", clamp_ceiling_always);
    /* S5-U12: Property 3 - result == 255 iff echo_us == 0. */
    TEST_CHECK("S5-U12", sentinel_iff_zero);

    /* S5-U13: representative example anchors (floor to 2, linear band, ceiling
     * clamp to 254, and the NO_ECHO sentinel) - concrete values alongside the
     * swept invariants. */
    {
        bool anchors_ok = true;

        if (us_echo_to_cm(0u)      != OBS_DISTANCE_NO_ECHO) { anchors_ok = false; } /* 1.5 */
        if (us_echo_to_cm(1u)      != 2u)                   { anchors_ok = false; } /* 1.3 floor */
        if (us_echo_to_cm(58u)     != 2u)                   { anchors_ok = false; } /* 1 cm -> floor 2 */
        if (us_echo_to_cm(116u)    != 2u)                   { anchors_ok = false; } /* exactly 2 cm */
        if (us_echo_to_cm(5800u)   != 100u)                 { anchors_ok = false; } /* 100 cm linear */
        if (us_echo_to_cm(1000000u) != OBS_DISTANCE_MAX_CM) { anchors_ok = false; } /* 1.2 clamp 254 */
        TEST_CHECK("S5-U13", anchors_ok);
    }
}

/* ==========================================================================
 * Task 2.5 - Property 7: packing round-trips.
 * --------------------------------------------------------------------------
 * Two independent packers must be lossless and lane-isolated:
 *
 *   obs_pack_motor_cmd(cmd, payload14) lays the 2-bit command into bits [15:14]
 *   and the 14-bit payload into bits [13:0]; obs_unpack_motor_cmd /
 *   obs_unpack_motor_payload must recover the original cmd (0..3) and payload
 *   (0..16383) with no field bleeding into its neighbour.
 *
 *   obs_pack_telemetry(profile, flags) lays four bytes into distinct lanes -
 *   [31:24] flags, [23:16] center_angle_deg, [15:8] closest_distance_cm,
 *   [7:0] approx_width_cm - with no truncation and no cross-field overlap; the
 *   obs_unpack_telemetry_* helpers must recover each lane byte exactly.
 *
 * This is a PROPERTY test. The motor-command round-trip is checked EXHAUSTIVELY
 * over all 4 commands x all 16384 payloads (every representable field value),
 * plus a randomized sweep that deliberately feeds OVER-WIDE command/payload
 * inputs to prove the pack masks each field to its lane (no corruption of the
 * neighbour). The telemetry round-trip sweeps every byte lane across 0..255 via
 * the PRNG and reassembles the word to prove the four lanes never swap or
 * overlap.
 *
 * **Validates: Requirements 3.3, 5.1, 5.2**
 * ========================================================================== */

#define OBS_P7_RAND_ITERATIONS  4000u

void test_packing_roundtrips(void)
{
    /* ---- Motor command: exhaustive over all commands x all payloads ------ */
    {
        bool     motor_roundtrip_ok = true;
        uint16_t cmd_i;
        uint32_t payload;

        for (cmd_i = 0u; cmd_i < 4u; cmd_i++) {
            for (payload = 0u; payload <= OBS_MOTOR_PAYLOAD_MASK; payload++) {
                uint16_t    packed = obs_pack_motor_cmd((motor_cmd_t)cmd_i,
                                                        (uint16_t)payload);
                motor_cmd_t got_cmd     = obs_unpack_motor_cmd(packed);
                uint16_t    got_payload = obs_unpack_motor_payload(packed);

                if (((uint16_t)got_cmd != cmd_i) ||
                    (got_payload != (uint16_t)payload)) {
                    motor_roundtrip_ok = false;
                }
            }
        }
        /* S5-U05: every command (0..3) x payload (0..16383) round-trips. */
        TEST_CHECK("S5-U05", motor_roundtrip_ok);
    }

    /* ---- Motor command: boundary + over-wide inputs stay lane-isolated --- */
    {
        bool     boundary_ok = true;
        uint16_t packed;

        /* Max command + max payload. */
        packed = obs_pack_motor_cmd(MOTOR_CMD_BYPASS_RIGHT, OBS_MOTOR_PAYLOAD_MASK);
        if ((obs_unpack_motor_cmd(packed) != MOTOR_CMD_BYPASS_RIGHT) ||
            (obs_unpack_motor_payload(packed) != OBS_MOTOR_PAYLOAD_MASK)) {
            boundary_ok = false;
        }

        /* Over-wide payload (> 14 bits) must be masked, never bleed into cmd. */
        {
            uint32_t iter;
            for (iter = 0u; iter < OBS_P7_RAND_ITERATIONS; iter++) {
                uint16_t    wide_cmd     = (uint16_t)obs_prng_range(0u, 0xFFFFu);
                uint16_t    wide_payload = (uint16_t)obs_prng_range(0u, 0xFFFFu);
                uint16_t    p            = obs_pack_motor_cmd((motor_cmd_t)wide_cmd,
                                                              wide_payload);
                motor_cmd_t exp_cmd      = (motor_cmd_t)(wide_cmd & OBS_MOTOR_CMD_MASK);
                uint16_t    exp_payload  = (uint16_t)(wide_payload &
                                                      OBS_MOTOR_PAYLOAD_MASK);

                if ((obs_unpack_motor_cmd(p) != exp_cmd) ||
                    (obs_unpack_motor_payload(p) != exp_payload)) {
                    boundary_ok = false;
                }
            }
        }
        /* S5-U06: boundary (max cmd + max payload) and over-wide inputs stay
         * masked to their own lane. */
        TEST_CHECK("S5-U06", boundary_ok);
    }

    /* ---- Telemetry: every byte lane swept 0..255, no truncation ---------- */
    {
        bool     telemetry_roundtrip_ok = true;
        uint32_t iter;

        for (iter = 0u; iter < OBS_P7_RAND_ITERATIONS; iter++) {
            obstacle_profile_t p;
            uint8_t            flags  = (uint8_t)obs_prng_range(0u, 255u);
            uint8_t            angle  = (uint8_t)obs_prng_range(0u, 255u);
            uint8_t            dist   = (uint8_t)obs_prng_range(0u, 255u);
            uint8_t            width  = (uint8_t)obs_prng_range(0u, 255u);
            uint32_t           packed;

            p.present             = true;
            p.center_angle_deg    = angle;
            p.closest_distance_cm = dist;
            p.approx_width_cm     = width;
            p.left_clearance_cm   = 0u; /* not part of the packed word */
            p.right_clearance_cm  = 0u; /* not part of the packed word */

            packed = obs_pack_telemetry(&p, flags);

            if ((obs_unpack_telemetry_flags(packed)              != flags) ||
                (obs_unpack_telemetry_center_angle_deg(packed)   != angle) ||
                (obs_unpack_telemetry_closest_distance_cm(packed) != dist) ||
                (obs_unpack_telemetry_approx_width_cm(packed)     != width)) {
                telemetry_roundtrip_ok = false;
            }
        }
        /* S5-U07: each telemetry lane (flags/angle/distance/width) round-trips
         * across the swept 0..255 range with no truncation. */
        TEST_CHECK("S5-U07", telemetry_roundtrip_ok);
    }

    /* ---- Telemetry: four distinct lane bytes reassemble with no lane swap - */
    {
        obstacle_profile_t p;
        uint32_t           packed;
        bool               layout_ok = true;

        /* Four deliberately-distinct lane bytes so any swap/overlap is caught. */
        p.present             = true;
        p.center_angle_deg    = 0x12u; /* [23:16] */
        p.closest_distance_cm = 0x34u; /* [15: 8] */
        p.approx_width_cm     = 0x56u; /* [ 7: 0] */
        p.left_clearance_cm   = 0u;
        p.right_clearance_cm  = 0u;

        packed = obs_pack_telemetry(&p, 0xABu /* [31:24] */);

        if (packed != 0xAB123456u) {
            layout_ok = false;
        }
        if ((obs_unpack_telemetry_flags(packed)               != 0xABu) ||
            (obs_unpack_telemetry_center_angle_deg(packed)    != 0x12u) ||
            (obs_unpack_telemetry_closest_distance_cm(packed) != 0x34u) ||
            (obs_unpack_telemetry_approx_width_cm(packed)     != 0x56u)) {
            layout_ok = false;
        }
        /* S5-U08: the four lanes reassemble to [31:24]|[23:16]|[15:8]|[7:0]
         * with no lane swap or overlap. */
        TEST_CHECK("S5-U08", layout_ok);
    }
}

/* ==========================================================================
 * Task 3.2 - Property 4: Filter rejects a single outlier.
 * --------------------------------------------------------------------------
 * filter_samples() takes the raw samples at one servo angle and reduces them
 * to a single filtered distance. Property 4 says: given >= 3 near-equal
 * samples (a consistent cluster) plus one wild outlier, the filtered result
 * must come from the consistent cluster and must NEVER be the outlier; and an
 * all-NO_ECHO input must return NO_ECHO.
 *
 * Why this holds for filter_samples(): with >= 3 equal cluster samples the
 * cluster value occurs >= 2 times, so it is the unique strict mode and is
 * returned; a single outlier can never reach a count of 2, so it can never be
 * selected as the mode, and (being a single distinct value) it is never the
 * lower-middle median either once a >=2 mode exists.
 *
 * This is a PROPERTY test: instead of a handful of literals it sweeps the input
 * space with a small deterministic PRNG (host-only, no external PBT library on
 * device) across many randomized cluster values, outlier values, counts, and
 * outlier positions, asserting the invariant on every generated case. The
 * generators are *smart*: they constrain to the valid distance range [2..254],
 * keep the outlier distinct from (and far from) the cluster, and keep the
 * cluster size >= 3 so the preconditions of Property 4 actually hold.
 *
 * **Validates: Requirements 1.6, 1.7, 2.2, 2.3, 2.4**
 * ========================================================================== */

/* Uses the shared deterministic xorshift32 PRNG (obs_prng_* above) so the
 * property run is reproducible on any host without <stdlib.h> rand() concerns. */

#define OBS_P4_ITERATIONS   2000u  /* randomized cases to exercise */
#define OBS_VALID_MIN_CM    2u     /* smallest valid distance (see us_echo_to_cm) */
#define OBS_VALID_MAX_CM    254u   /* OBS_DISTANCE_MAX_CM, below the 255 sentinel */

void test_filter_samples_props(void)
{
    uint32_t iter;
    bool     outlier_rejected_always = true; /* result never equals the outlier */
    bool     cluster_returned_always = true; /* result is the cluster value     */
    uint32_t cases_run               = 0u;

    for (iter = 0u; iter < OBS_P4_ITERATIONS; iter++) {
        uint8_t  samples[16];
        uint8_t  n;
        uint8_t  cluster_size;
        uint8_t  cluster_value;
        uint8_t  outlier_value;
        uint8_t  outlier_pos;
        uint8_t  fill_pos;
        uint8_t  k;
        uint8_t  result;

        /* --- smart generators -------------------------------------------- */
        /* Cluster size >= 3 (Property 4 precondition), leaving room for the one
         * outlier within the fixed buffer. */
        cluster_size = (uint8_t)obs_prng_range(3u, 15u);
        n            = (uint8_t)(cluster_size + 1u); /* cluster + exactly 1 outlier */

        cluster_value = (uint8_t)obs_prng_range(OBS_VALID_MIN_CM, OBS_VALID_MAX_CM);

        /* Outlier must be a VALID, WILD, and DISTINCT value - "wild" meaning far
         * from the cluster so it is unmistakably an outlier, and never the
         * NO_ECHO sentinel (that path is covered by the all-NO_ECHO case below).
         * Pick the far end of the range from the cluster to guarantee distance. */
        if (cluster_value <= 128u) {
            /* cluster is low -> outlier is high and >= 50 cm away */
            outlier_value = (uint8_t)obs_prng_range(
                (uint32_t)cluster_value + 50u, OBS_VALID_MAX_CM);
        } else {
            /* cluster is high -> outlier is low and >= 50 cm away */
            outlier_value = (uint8_t)obs_prng_range(
                OBS_VALID_MIN_CM, (uint32_t)cluster_value - 50u);
        }

        /* Place the single outlier at a random index so position is exercised. */
        outlier_pos = (uint8_t)obs_prng_range(0u, (uint32_t)(n - 1u));

        fill_pos = 0u;
        for (k = 0u; k < n; k++) {
            if (k == outlier_pos) {
                samples[k] = outlier_value;
            } else {
                samples[k] = cluster_value;
                fill_pos++;
            }
        }
        (void)fill_pos;

        /* --- the invariant ----------------------------------------------- */
        result = filter_samples(samples, n);

        cases_run++;
        if (result == outlier_value) {
            outlier_rejected_always = false; /* must NEVER be the outlier */
        }
        if (result != cluster_value) {
            cluster_returned_always = false; /* must come from the cluster  */
        }
    }

    /* S5-U30: the result is always the consistent-cluster value across the
     * whole randomized sweep (and therefore never the single wild outlier). */
    TEST_CHECK("S5-U30", cluster_returned_always);
    /* S5-U31: stated directly - the single outlier is never selected. */
    TEST_CHECK("S5-U31", outlier_rejected_always);
    /* Guard: the sweep actually generated the intended number of cases. */
    TEST_CHECK("S5-U32", cases_run == OBS_P4_ITERATIONS);

    /* All-NO_ECHO -> NO_ECHO, for several n including the n == 0 edge. (1.7, 2.4) */
    {
        uint8_t all_no_echo[8];
        uint8_t i;
        bool    all_no_echo_ok = true;

        for (i = 0u; i < 8u; i++) {
            all_no_echo[i] = OBS_DISTANCE_NO_ECHO;
        }
        for (i = 1u; i <= 8u; i++) {
            if (filter_samples(all_no_echo, i) != OBS_DISTANCE_NO_ECHO) {
                all_no_echo_ok = false;
            }
        }
        /* n == 0 (empty / NULL) also yields the sentinel. */
        if (filter_samples(all_no_echo, 0u) != OBS_DISTANCE_NO_ECHO) {
            all_no_echo_ok = false;
        }
        if (filter_samples(0, 0u) != OBS_DISTANCE_NO_ECHO) {
            all_no_echo_ok = false;
        }
        TEST_CHECK("S5-U33", all_no_echo_ok);
    }

    /* Mixed: a valid cluster with a wild outlier AND some NO_ECHO noise still
     * returns the cluster value (NO_ECHO samples are discarded first). */
    {
        uint8_t mixed[7] = {
            OBS_DISTANCE_NO_ECHO, 40u, 40u, 200u, 40u,
            OBS_DISTANCE_NO_ECHO, 40u
        };
        TEST_CHECK("S5-U34", filter_samples(mixed, 7u) == 40u);
    }
}

/* ==========================================================================
 * Task 4.2 - Property 6: planning picks the higher clearance.
 * --------------------------------------------------------------------------
 * plan_bypass() returns MOTOR_CMD_BYPASS_LEFT IF AND ONLY IF the estimated
 * left clearance is strictly greater than the estimated right clearance; in
 * every other case - left < right, or the left == right tie - it returns
 * MOTOR_CMD_BYPASS_RIGHT. The decision is also total: it is always one of the
 * two bypass sides, never STOP/FORWARD.
 *
 * This is a PROPERTY test over the FULL left x right clearance space: it sweeps
 * all 256 x 256 uint8_t pairs and asserts the iff on every pair, so it covers
 * every possible combination of clearances rather than a few examples. The
 * profile fields other than the two clearances are fixed, isolating the
 * left-vs-right comparison that Property 6 is about.
 *
 * **Validates: Requirements 3.1, 3.2**
 * ========================================================================== */
void test_plan_bypass_props(void)
{
    obstacle_profile_t p;
    bool               iff_holds      = true; /* LEFT iff left strictly > right */
    bool               total_decision = true; /* result is always a bypass side */
    uint16_t           left;
    uint16_t           right;

    /* Non-clearance fields must not influence the side decision; fix them. */
    p.present             = true;
    p.closest_distance_cm = 10u;
    p.center_angle_deg    = 90u;
    p.approx_width_cm     = 12u;

    for (left = 0u; left <= 0xFFu; left++) {
        for (right = 0u; right <= 0xFFu; right++) {
            motor_cmd_t cmd;
            bool        expect_left = (left > right);

            p.left_clearance_cm  = (uint8_t)left;
            p.right_clearance_cm = (uint8_t)right;

            cmd = plan_bypass(&p);

            /* P6 iff, both directions: strictly-greater left -> LEFT;
             * otherwise (including the equal-clearance tie) -> RIGHT. */
            if (expect_left) {
                if (cmd != MOTOR_CMD_BYPASS_LEFT) {
                    iff_holds = false;
                }
            } else {
                if (cmd != MOTOR_CMD_BYPASS_RIGHT) {
                    iff_holds = false;
                }
            }

            /* Totality: never STOP/FORWARD, always a concrete bypass side. */
            if ((cmd != MOTOR_CMD_BYPASS_LEFT) &&
                (cmd != MOTOR_CMD_BYPASS_RIGHT)) {
                total_decision = false;
            }
        }
    }

    TEST_CHECK("S5-U20", iff_holds);       /* P6: LEFT iff left > right     */
    TEST_CHECK("S5-U21", total_decision);  /* decision is total (a bypass)  */
}

/* ==========================================================================
 * Task 4.4 - unit tests for estimate_profile.
 * --------------------------------------------------------------------------
 * estimate_profile(readings, count, lo, hi) reduces an injected array of
 * filtered per-angle readings to one obstacle_profile_t ESTIMATE. These
 * EXAMPLE/edge-case unit tests cover: closest-distance + center-angle
 * selection (2.10, 2.11), the smallest-angle tie-break, left/right clearance
 * as the min valid distance over each side of center (2.12 left = angles >
 * center, 2.13 right = angles < center), width-estimate clamping to 0..254
 * (2.9), and the `present` threshold exactly at the SAFE_LIMIT_CM (30 cm)
 * boundary and one cm past it. NO_ECHO readings are ignored for every
 * statistic.
 *
 * _Requirements: 2.9, 2.10, 2.11, 2.12, 2.13_
 * ========================================================================== */
void test_estimate_profile_unit(void)
{
    /* S5-U40: closest distance + center angle selection; NO_ECHO ignored. */
    {
        const angle_reading_t readings[] = {
            { 30u,  80u },
            { 60u,  50u },
            { 90u,  25u },                   /* closest valid return */
            {120u,  OBS_DISTANCE_NO_ECHO },   /* ignored for all stats */
            {150u,  40u },
        };
        obstacle_profile_t p = estimate_profile(readings, 5u, 30u, 150u);

        TEST_CHECK("S5-U40", p.closest_distance_cm == 25u);  /* 2.10 */
        TEST_CHECK("S5-U40", p.center_angle_deg == 90u);     /* 2.11 */
    }

    /* S5-U41: equal-closest tie-break resolves to the smallest angle. */
    {
        const angle_reading_t readings[] = {
            { 60u,  25u },   /* first occurrence of the minimum */
            { 90u,  40u },
            {120u,  25u },   /* also minimum, but a larger angle */
        };
        obstacle_profile_t p = estimate_profile(readings, 3u, 30u, 150u);

        TEST_CHECK("S5-U41", p.closest_distance_cm == 25u);
        TEST_CHECK("S5-U41", p.center_angle_deg == 60u);     /* 2.11 */
    }

    /* S5-U42: left/right clearance are the min valid distance over each side.
     * center = 90 (closest 20); left (angle > 90) = {120:70,150:35} -> 35;
     * right (angle < 90) = {30:90,60:45} -> 45. */
    {
        const angle_reading_t readings[] = {
            { 30u,  90u },
            { 60u,  45u },   /* right-side min */
            { 90u,  20u },   /* center / closest */
            {120u,  70u },
            {150u,  35u },   /* left-side min  */
        };
        obstacle_profile_t p = estimate_profile(readings, 5u, 30u, 150u);

        TEST_CHECK("S5-U42", p.center_angle_deg == 90u);
        TEST_CHECK("S5-U42", p.left_clearance_cm == 35u);    /* 2.12 */
        TEST_CHECK("S5-U42", p.right_clearance_cm == 45u);   /* 2.13 */
    }

    /* S5-U43: a side with no reading defaults to fully-open MAX_CM. Closest is
     * at the lowest angle (30), so nothing lies to its right. */
    {
        const angle_reading_t readings[] = {
            { 30u,  20u },   /* center / closest; nothing to the right */
            { 60u,  60u },
            { 90u,  50u },   /* left-side min */
        };
        obstacle_profile_t p = estimate_profile(readings, 3u, 30u, 150u);

        TEST_CHECK("S5-U43", p.center_angle_deg == 30u);
        TEST_CHECK("S5-U43", p.right_clearance_cm == OBS_DISTANCE_MAX_CM);
        TEST_CHECK("S5-U43", p.left_clearance_cm == 50u);
    }

    /* S5-U44: width estimate clamps to 0..254. Five near-equal 254 cm returns
     * span the full window; the linear estimate overflows 254 and saturates. */
    {
        const angle_reading_t readings[] = {
            { 30u,  254u },
            { 60u,  254u },
            { 90u,  254u },
            {120u,  254u },
            {150u,  254u },
        };
        obstacle_profile_t p = estimate_profile(readings, 5u, 30u, 150u);

        TEST_CHECK("S5-U44", p.approx_width_cm == OBS_DISTANCE_MAX_CM); /* 2.9 */
        TEST_CHECK("S5-U44", p.approx_width_cm <= OBS_DISTANCE_MAX_CM);
    }

    /* S5-U45: a single in-window return gives a zero-span (0 cm) width. */
    {
        const angle_reading_t readings[] = {
            { 90u,  40u },
        };
        obstacle_profile_t p = estimate_profile(readings, 1u, 30u, 150u);

        TEST_CHECK("S5-U45", p.approx_width_cm == 0u);
    }

    /* S5-U46: `present` is true AT the boundary (closest == SAFE_LIMIT_CM). */
    {
        const angle_reading_t readings[] = {
            { 90u,  30u },   /* exactly SAFE_LIMIT_CM */
        };
        obstacle_profile_t p = estimate_profile(readings, 1u, 30u, 150u);

        TEST_CHECK("S5-U46", p.closest_distance_cm == 30u);
        TEST_CHECK("S5-U46", p.present == true);
    }

    /* S5-U47: `present` is false one cm past the boundary (closest == 31). */
    {
        const angle_reading_t readings[] = {
            { 90u,  31u },   /* SAFE_LIMIT_CM + 1 */
        };
        obstacle_profile_t p = estimate_profile(readings, 1u, 30u, 150u);

        TEST_CHECK("S5-U47", p.closest_distance_cm == 31u);
        TEST_CHECK("S5-U47", p.present == false);
    }

    /* S5-U48: an all-NO_ECHO window reports not-present with safe defaults. */
    {
        const angle_reading_t readings[] = {
            { 30u,  OBS_DISTANCE_NO_ECHO },
            { 90u,  OBS_DISTANCE_NO_ECHO },
            {150u,  OBS_DISTANCE_NO_ECHO },
        };
        obstacle_profile_t p = estimate_profile(readings, 3u, 30u, 150u);

        TEST_CHECK("S5-U48", p.present == false);
        TEST_CHECK("S5-U48", p.closest_distance_cm == OBS_DISTANCE_MAX_CM);
        TEST_CHECK("S5-U48", p.approx_width_cm == 0u);
        TEST_CHECK("S5-U48", p.left_clearance_cm == OBS_DISTANCE_MAX_CM);
        TEST_CHECK("S5-U48", p.right_clearance_cm == OBS_DISTANCE_MAX_CM);
    }
}

/* ==========================================================================
 * Task 7.2 - Property 5: Servo angle bounded.
 * --------------------------------------------------------------------------
 * servo_clamp_angle(angle_deg, &out_of_range) is the PURE clamp factored out of
 * the servo driver (task 7.1). Correctness Property 5 says: for ANY requested
 * angle, the effective angle handed to the PWM is always within
 * [SRV_MIN_DEG, SRV_MAX_DEG] = [30, 150]; a request below 30 or above 150 is
 * clamped to the nearest bound AND reported out-of-range (so the driver latches
 * FAULT_SERVO), while an in-range request passes through unchanged and is NOT
 * flagged.
 *
 * This is a PROPERTY test. Because the request is a uint8_t, the whole input
 * space is only 0..255, so the sweep is EXHAUSTIVE over every representable
 * requested angle rather than sampled. For each request it asserts:
 *   - the clamped result is always in [30, 150] (the bound invariant);
 *   - out_of_range is true IFF the request was < 30 or > 150;
 *   - an in-range request (30..150) is returned unchanged;
 *   - a below-range request clamps to exactly 30, an above-range to exactly 150.
 * The NULL out_of_range argument is also exercised (the clamp must still work
 * and must not dereference NULL).
 *
 * **Validates: Requirements 2.5, 3.11**
 * ========================================================================== */
void test_servo_clamp_props(void)
{
    bool     bounded_always      = true; /* result always in [30,150]        */
    bool     flag_iff_out        = true; /* out_of_range iff request outside  */
    bool     in_range_unchanged  = true; /* 30..150 passes through unchanged  */
    bool     clamps_to_bound     = true; /* below->30, above->150            */
    uint16_t req;                        /* 0..255 (uint16_t avoids wrap)     */

    for (req = 0u; req <= 255u; req++) {
        uint8_t angle = (uint8_t)req;
        bool    out_of_range = false;
        uint8_t clamped      = servo_clamp_angle(angle, &out_of_range);
        bool    expect_out   = (angle < (uint8_t)SRV_MIN_DEG) ||
                               (angle > (uint8_t)SRV_MAX_DEG);

        /* Property 5: the effective angle is always inside the servo range. */
        if ((clamped < (uint8_t)SRV_MIN_DEG) ||
            (clamped > (uint8_t)SRV_MAX_DEG)) {
            bounded_always = false;
        }

        /* out_of_range reported exactly when the request was outside [30,150]. */
        if (out_of_range != expect_out) {
            flag_iff_out = false;
        }

        if (expect_out) {
            /* Below range clamps to the low bound, above range to the high. */
            if (angle < (uint8_t)SRV_MIN_DEG) {
                if (clamped != (uint8_t)SRV_MIN_DEG) {
                    clamps_to_bound = false;
                }
            } else { /* angle > SRV_MAX_DEG */
                if (clamped != (uint8_t)SRV_MAX_DEG) {
                    clamps_to_bound = false;
                }
            }
        } else {
            /* In-range request must pass through completely unchanged. */
            if (clamped != angle) {
                in_range_unchanged = false;
            }
        }
    }

    /* S5-U50: Property 5 - the clamped (effective) angle is always in [30,150]
     * for every requested angle 0..255. */
    TEST_CHECK("S5-U50", bounded_always);
    /* S5-U51: out_of_range is true iff the request was < 30 or > 150. */
    TEST_CHECK("S5-U51", flag_iff_out);
    /* S5-U52: an in-range request (30..150) is returned unchanged. */
    TEST_CHECK("S5-U52", in_range_unchanged);
    /* S5-U53: a below-range request clamps to 30, an above-range to 150. */
    TEST_CHECK("S5-U53", clamps_to_bound);

    /* S5-U54: the clamp works (and does not dereference) with a NULL
     * out_of_range pointer, across the representative boundary set. */
    {
        bool null_ptr_ok = true;

        if (servo_clamp_angle(0u,   0) != (uint8_t)SRV_MIN_DEG) { null_ptr_ok = false; }
        if (servo_clamp_angle(29u,  0) != (uint8_t)SRV_MIN_DEG) { null_ptr_ok = false; }
        if (servo_clamp_angle(30u,  0) != (uint8_t)SRV_MIN_DEG) { null_ptr_ok = false; }
        if (servo_clamp_angle(90u,  0) != 90u)                  { null_ptr_ok = false; }
        if (servo_clamp_angle(150u, 0) != (uint8_t)SRV_MAX_DEG) { null_ptr_ok = false; }
        if (servo_clamp_angle(151u, 0) != (uint8_t)SRV_MAX_DEG) { null_ptr_ok = false; }
        if (servo_clamp_angle(255u, 0) != (uint8_t)SRV_MAX_DEG) { null_ptr_ok = false; }
        TEST_CHECK("S5-U54", null_ptr_ok);
    }
}

/* ==========================================================================
 * Task 9.2 - Property 8: the >= 60 ms HC-SR04 cycle floor.
 * --------------------------------------------------------------------------
 * Property 8 (Cycle floor): every pair of CONSECUTIVE HC-SR04 triggers is
 * separated by at least US_CYCLE_MS (60 ms). This is the timing invariant that
 * resolves the 10 ms-vs-60 ms conflict (req 5.3, 5.4): the sensing cadence is
 * governed by the hardware's >= 60 ms minimum trigger-to-trigger interval.
 *
 * Unlike the other tests in this file (which exercise PURE logic), this one
 * measures the REAL aim_and_read() in obstacle_avoidance.c so the actual
 * ordering of "fire a trigger / wait" is what gets checked. aim_and_read()
 * pulls FreeRTOS task.h, so the host build supplies a host_stubs/task.h shim
 * (and this file provides the two hardware symbols the scan TU references,
 * servo_set_angle and us_measure_echo_us, as no-op/link-only stubs).
 *
 * The measurement uses the subsystem's two swappable timing seams (declared in
 * obstacle_scan.h) WITHOUT any real hardware or scheduler:
 *   - a DELAY hook that advances a simulated millisecond clock by `ms` (instead
 *     of sleeping), so SRV_SETTLE_MS and each US_CYCLE_MS gap move virtual time;
 *   - a MEASURE hook that stands in for one HC-SR04 ranging cycle: it RECORDS
 *     the current simulated timestamp (that is the moment a trigger is issued)
 *     and returns a scripted echo high-time.
 * aim_and_read() calls the measure seam once per sample, so the recorded
 * timestamps ARE the trigger times; asserting every consecutive pair differs by
 * >= 60 ms directly verifies Property 8 over a full angle's sampling loop.
 *
 * **Validates: Requirements 5.3, 5.4**
 * ========================================================================== */

/* ---- Link-only / no-op host stubs for the two hardware symbols that the scan
 * translation unit (obstacle_avoidance.c) references. The servo call inside
 * aim_and_read() does nothing on the host; the default measure seam points at
 * us_measure_echo_us, but the test installs its own measure hook so the real
 * body is never invoked - these only need to resolve at link time. ---------- */
void servo_set_angle(uint8_t angle_deg)
{
    (void)angle_deg;
}

bool servo_fault(void)
{
    return false; /* host stub: no real servo, so never faulted */
}

uint32_t us_measure_echo_us(void)
{
    return 0u; /* never reached: the test installs its own measure hook */
}

/* ---- Link-only host stubs for the two init symbols that obstacle_setup()
 * references (task 13.1). obstacle_setup() is only reached from the demo main()
 * in obstacle_avoidance.c (renamed via -Dmain=... and never called by this
 * runner), so these just need to resolve at link time on the host. ---------- */
void us_pins_init(void)
{
    /* host stub: no real HC-SR04 pins to configure */
}

void servo_init(void)
{
    /* host stub: no real servo PWM slice to configure */
}

/* Link-only host stub for Buddy 3's line query, referenced by line_recovery()
 * (task 11.2) in obstacle_avoidance.c. The current host tests exercise only
 * aim_and_read() (which never calls it), so a fixed "line found" return keeps
 * the TU linking without a real line sensor; a dedicated recovery host test
 * would install its own scripted behaviour. */
bool line_detected(void)
{
    return true;
}

/* Link-only host stub for Buddy 4's terrain hump flag, referenced by the
 * obstacle_task state machine (task 13.2) in obstacle_avoidance.c. The current
 * host tests exercise only the pure logic and aim_and_read() (none of which call
 * it), and obstacle_task is an infinite loop that is never run off-device, so a
 * fixed HUMP_NONE keeps the TU linking without a real terrain monitor. */
hump_state_t terrain_hump_state(void)
{
    return HUMP_NONE;
}

/* ---- Simulated clock + recorded trigger timestamps shared by the two hooks - */
#define OBS_MAX_TRIGGERS  64u

static uint32_t g_sim_now_ms = 0u;              /* advanced by the delay hook  */
static uint32_t g_trigger_ms[OBS_MAX_TRIGGERS]; /* timestamp of each trigger   */
static uint8_t  g_trigger_count = 0u;           /* number of triggers recorded */

/* Delay seam: advance virtual time by `ms` instead of sleeping. This is what
 * makes SRV_SETTLE_MS and each inter-sample US_CYCLE_MS gap move the clock. */
static void sim_delay_ms(uint32_t ms)
{
    g_sim_now_ms += ms;
}

/* Measure seam: one simulated ranging cycle. Records the trigger timestamp
 * (the current simulated clock) then returns a scripted echo high-time. The
 * returned value is irrelevant to the timing invariant, so a fixed in-range
 * echo is fine (5800 us -> 100 cm). */
static uint32_t sim_measure_record(void)
{
    if (g_trigger_count < OBS_MAX_TRIGGERS) {
        g_trigger_ms[g_trigger_count] = g_sim_now_ms;
        g_trigger_count++;
    }
    return 5800u; /* any non-zero echo; value does not affect trigger spacing */
}

void test_cycle_floor_props(void)
{
    uint8_t  i;
    bool     floor_holds   = true;  /* every consecutive gap >= US_CYCLE_MS   */
    bool     settle_before = true;  /* first trigger only after SRV_SETTLE_MS */

    /* Reset the simulated clock and the recorded-trigger log, then install the
     * instrumented seams in place of the real HC-SR04 / vTaskDelay. */
    g_sim_now_ms    = 0u;
    g_trigger_count = 0u;

    obs_set_delay_hook(sim_delay_ms);
    obs_set_measure_hook(sim_measure_record);

    /* Exercise the REAL per-angle sampling loop at a representative angle. It
     * fires exactly READINGS_PER_ANGLE triggers, each followed by a US_CYCLE_MS
     * delay, after an initial SRV_SETTLE_MS settle. */
    (void)aim_and_read(90u);

    /* S5-U60: Property 8 - every pair of consecutive triggers is >= 60 ms apart
     * (the HC-SR04 cycle floor). Checked over the whole recorded sweep. */
    for (i = 1u; i < g_trigger_count; i++) {
        if ((g_trigger_ms[i] - g_trigger_ms[i - 1u]) < (uint32_t)US_CYCLE_MS) {
            floor_holds = false;
        }
    }
    TEST_CHECK("S5-U60", floor_holds);

    /* S5-U61: the sampling loop fired exactly READINGS_PER_ANGLE triggers, so
     * the cycle-floor sweep above actually covered every sample (coverage
     * guard - an empty/short sweep must not trivially "pass"). */
    TEST_CHECK("S5-U61", g_trigger_count == (uint8_t)READINGS_PER_ANGLE);

    /* S5-U62: the first trigger happens only AFTER the servo settle window
     * (req 2.6 / 5.3) - no ranging during servo travel; and the exact expected
     * timeline holds: trigger k lands at SRV_SETTLE_MS + k * US_CYCLE_MS. */
    if (g_trigger_count >= 1u) {
        if (g_trigger_ms[0] < (uint32_t)SRV_SETTLE_MS) {
            settle_before = false;
        }
        for (i = 0u; i < g_trigger_count; i++) {
            uint32_t expected = (uint32_t)SRV_SETTLE_MS +
                                ((uint32_t)i * (uint32_t)US_CYCLE_MS);
            if (g_trigger_ms[i] != expected) {
                settle_before = false;
            }
        }
    } else {
        settle_before = false;
    }
    TEST_CHECK("S5-U62", settle_before);

    /* Restore the real seams so later tests / reruns see production behaviour. */
    obs_set_delay_hook(NULL);
    obs_set_measure_hook(NULL);
}

/* ==========================================================================
 * Task 13.3 - unit test for the hump-suppression transition guard.
 * --------------------------------------------------------------------------
 * obs_should_escalate(distance_cm, hump) is the PURE form of obstacle_task's
 * IDLE_MONITORING escalation decision (factored out in task 13.3). It returns
 * true IFF the terrain is clear of a hump (HUMP_NONE) AND the idle reading is a
 * real echo (!= OBS_DISTANCE_NO_ECHO) strictly inside SAFE_LIMIT_CM (30 cm).
 *
 * These EXAMPLE/edge-case unit tests pin the two requirements the guard exists
 * for:
 *   - req 1.11: while a hump is active (any hump != HUMP_NONE) a proximity
 *     trigger is IGNORED, so the guard must be false for EVERY non-NONE hump
 *     state regardless of distance - even a would-be-escalating near reading.
 *   - req 1.8: with HUMP_NONE the guard is true ONLY for a real echo strictly
 *     below SAFE_LIMIT_CM; it is false at/above the limit and for the NO_ECHO
 *     sentinel.
 *
 * _Requirements: 1.8, 1.11_
 * ========================================================================== */
void test_hump_suppression_guard(void)
{
    /* The non-NONE hump states to sweep (req 1.11: all must suppress). */
    const hump_state_t active_humps[] = {
        HUMP_INCLINE, HUMP_LEVEL, HUMP_DECLINE
    };

    /* S5-U70: req 1.11 - while ANY hump is active the guard is false for EVERY
     * distance, including a near reading that would escalate under HUMP_NONE.
     * Swept exhaustively over all distances 0..255 x every non-NONE hump. */
    {
        bool suppressed_always = true;
        uint8_t h;
        uint16_t dist; /* uint16_t avoids 8-bit wrap on the 0..255 sweep */

        for (h = 0u; h < (uint8_t)(sizeof(active_humps) /
                                   sizeof(active_humps[0])); h++) {
            for (dist = 0u; dist <= 255u; dist++) {
                if (obs_should_escalate((uint8_t)dist, active_humps[h])) {
                    suppressed_always = false;
                }
            }
        }
        TEST_CHECK("S5-U70", suppressed_always);
    }

    /* S5-U71: req 1.8 - with HUMP_NONE the guard is true for a real echo
     * strictly below SAFE_LIMIT_CM, and false at/above the limit. Swept over
     * every distance 0..254 (the NO_ECHO sentinel 255 is covered by S5-U73). */
    {
        bool threshold_ok = true;
        uint16_t dist;

        for (dist = 0u; dist < (uint16_t)OBS_DISTANCE_NO_ECHO; dist++) {
            bool expect = (dist < (uint16_t)SAFE_LIMIT_CM);
            if (obs_should_escalate((uint8_t)dist, HUMP_NONE) != expect) {
                threshold_ok = false;
            }
        }
        TEST_CHECK("S5-U71", threshold_ok);
    }

    /* S5-U72: req 1.8 boundary anchors with HUMP_NONE - just below the limit
     * escalates; exactly at the limit does NOT (strict '<'); just above does
     * not either. */
    {
        bool boundary_ok = true;

        if (obs_should_escalate((uint8_t)(SAFE_LIMIT_CM - 1u), HUMP_NONE) != true) {
            boundary_ok = false;                 /* 29 cm -> escalate */
        }
        if (obs_should_escalate((uint8_t)SAFE_LIMIT_CM, HUMP_NONE) != false) {
            boundary_ok = false;                 /* 30 cm -> NOT (strict <) */
        }
        if (obs_should_escalate((uint8_t)(SAFE_LIMIT_CM + 1u), HUMP_NONE) != false) {
            boundary_ok = false;                 /* 31 cm -> NOT */
        }
        if (obs_should_escalate(2u, HUMP_NONE) != true) {
            boundary_ok = false;                 /* near valid min -> escalate */
        }
        TEST_CHECK("S5-U72", boundary_ok);
    }

    /* S5-U73: the NO_ECHO sentinel (255) never escalates, even with HUMP_NONE -
     * a timeout is not a real return (so this also confirms the sentinel is not
     * treated as a far-but-valid reading). */
    {
        TEST_CHECK("S5-U73",
                   obs_should_escalate(OBS_DISTANCE_NO_ECHO, HUMP_NONE) == false);
    }
}

/* ==========================================================================
 * Host runner.
 * --------------------------------------------------------------------------
 * Calls every registered test function. Guarded with OBSTACLE_TEST_HOST_MAIN so
 * that when task 5's canonical harness provides the real entry point this file
 * contributes only its test functions (compile with -DOBSTACLE_TEST_HOST_MAIN=0
 * to drop this main). Returns non-zero if any TEST_CHECK failed, for CI use.
 * ========================================================================== */
#ifndef OBSTACLE_TEST_HOST_MAIN
#define OBSTACLE_TEST_HOST_MAIN 1
#endif

#if OBSTACLE_TEST_HOST_MAIN
int main(void)
{
    test_us_echo_to_cm_props();    /* task 2.2 - Properties 1/2/3 */
    test_packing_roundtrips();     /* task 2.5 - Property 7       */
    test_filter_samples_props();   /* task 3.2 - Property 4       */
    test_plan_bypass_props();      /* task 4.2 - Property 6       */
    test_estimate_profile_unit();  /* task 4.4 - estimate_profile unit tests */
    test_servo_clamp_props();      /* task 7.2 - Property 5 (servo bounded)   */
    test_cycle_floor_props();      /* task 9.2 - Property 8 (>= 60 ms floor)  */
    test_hump_suppression_guard(); /* task 13.3 - hump-suppression guard      */

    printf("[TEST] subsystem5 host: %u passed, %u failed\n",
           g_test_pass_count, g_test_fail_count);
    return (g_test_fail_count == 0u) ? 0 : 1;
}
#endif /* OBSTACLE_TEST_HOST_MAIN */
