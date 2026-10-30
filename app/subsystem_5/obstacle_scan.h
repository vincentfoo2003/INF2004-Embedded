#ifndef OBSTACLE_SCAN_H
#define OBSTACLE_SCAN_H

/*
 * Ultrasonic Obstacle Profiling (Buddy 5 / Subsystem 5) - scan pipeline
 * (INTERNAL header).
 *
 * This header exposes the aimed-and-filtered reading pipeline to the rest of
 * the subsystem (the coarse/fine scans and the obstacle_task state machine in
 * tasks 10-13 call aim_and_read) and to the off-device timing test (task 9.2).
 * It is NOT part of the public subsystem API in obstacle_avoidance.h - those
 * functions stay the only externally published surface.
 *
 * The pipeline is deliberately split from its two side-effecting primitives
 * (ranging a single HC-SR04 cycle, and sleeping between cycles) through two
 * swappable seams (see the "timing seam" section). On device both seams point
 * at the real hardware / FreeRTOS calls; a host test can install its own hooks
 * to (a) feed a scripted sample sequence and (b) RECORD the simulated timestamp
 * of every trigger, which is exactly what task 9.2 (Property 8: >= 60 ms cycle
 * floor) needs to observe the inter-trigger spacing WITHOUT real hardware.
 *
 * Conventions: Barr-C style - snake_case functions, #define macros, header
 * guards.
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_avoidance.h"     /* OBS_DISTANCE_NO_ECHO, QueueHandle_t */
#include "obstacle_integration.h"   /* READINGS_PER_ANGLE, US_CYCLE_MS, ... */
#include "obstacle_logic.h"         /* obstacle_profile_t (send_telemetry arg) */

/* --------------------------------------------------------------------------
 * Task 9.1: the aimed-and-filtered reading pipeline
 * --------------------------------------------------------------------------
 * aim_and_read() produces exactly one filtered distance at a single servo
 * angle:
 *   1. servo_set_angle(angle_deg)                 (clamped to [30,150] inside)
 *   2. wait SRV_SETTLE_MS (200 ms) so the servo physically settles before any
 *      ranging (req 2.6; within the 150..300 ms window)
 *   3. take exactly READINGS_PER_ANGLE (5) samples; EACH sample runs one
 *      HC-SR04 ranging cycle (via the measure seam) and is followed by a
 *      US_CYCLE_MS (>= 60 ms) delay, so every pair of consecutive triggers is
 *      separated by at least 60 ms (req 5.3, 5.4 - the cycle floor verified by
 *      task 9.2)
 *   4. convert each raw echo-high-time with us_echo_to_cm()
 *   5. reduce the 5 converted samples to one value with filter_samples()
 *      (mode when any value repeats, else median; all-NO_ECHO -> NO_ECHO)
 *
 * It also maintains the consecutive-NO_ECHO streak used for fault detection
 * (task 10.1 / 12.2): the streak is incremented when the filtered result for
 * this angle is NO_ECHO and reset to 0 on any valid filtered reading.
 *
 * Returns the filtered distance in cm (2..254) or OBS_DISTANCE_NO_ECHO (255).
 * _Requirements: 1.6, 1.7, 2.2, 2.3, 2.4, 2.6, 5.3, 5.4_
 */
uint8_t aim_and_read(uint8_t angle_deg);

/* --------------------------------------------------------------------------
 * Consecutive-NO_ECHO fault counter (shared with tasks 10.1 / 12.2)
 * --------------------------------------------------------------------------
 * aim_and_read() bumps this streak on an all-NO_ECHO angle and clears it on a
 * valid reading. The scan / fault logic reads it to decide when to latch
 * FAULT_NO_ECHO (FAULT_NO_ECHO_STREAK == 3 consecutive timeouts, req 5.7).
 */

/* Current number of consecutive angles that filtered to NO_ECHO. */
uint8_t obs_no_echo_streak(void);

/* Reset the consecutive-NO_ECHO streak to 0 (e.g. at the start of a new scan). */
void obs_reset_no_echo_streak(void);

/* --------------------------------------------------------------------------
 * Timing / ranging seams (test instrumentation for task 9.2)
 * --------------------------------------------------------------------------
 * On device these default to the real HC-SR04 ranging cycle and a FreeRTOS
 * vTaskDelay. A host test overrides them to inject a scripted sample stream and
 * to record the simulated millisecond timestamp of every trigger so the >= 60
 * ms cycle floor (Property 8) is observable off-device.
 *
 * NOTE: these hooks exist purely to keep aim_and_read()'s timing testable; the
 * production build never installs them, so the default behaviour is unchanged.
 */

/* Measure seam: run one ranging cycle and return the ECHO high-time in us
 * (0 == NO_ECHO), matching us_measure_echo_us(). */
typedef uint32_t (*us_measure_fn_t)(void);

/* Delay seam: sleep for the given number of milliseconds. The default wraps
 * vTaskDelay(pdMS_TO_TICKS(ms)); a host test advances a simulated clock. */
typedef void (*scan_delay_fn_t)(uint32_t ms);

/* Install a custom ranging hook (NULL restores the real us_measure_echo_us). */
void obs_set_measure_hook(us_measure_fn_t hook);

/* Install a custom delay hook (NULL restores the real vTaskDelay wrapper). */
void obs_set_delay_hook(scan_delay_fn_t hook);

/* --------------------------------------------------------------------------
 * Task 12.1: non-blocking telemetry post + telemetry-queue plumbing
 * --------------------------------------------------------------------------
 * send_telemetry() builds the obstacle TelemetryData32 message (id =
 * TELEMETRY_ID_OBSTACLE, data = obs_pack_telemetry(profile, flags)) and posts
 * it to the subsystem's telemetry queue with a 0-tick, non-blocking try-post
 * (req 5.5). It never blocks the scan loop: if the queue is full the message is
 * discarded, FAULT_QUEUE_FULL is latched (req 5.6), and the function returns
 * without any synchronous retry.
 *
 * The destination queue is stored by obstacle_task_start() (task 13.1) via the
 * internal setter obs_set_telemetry_queue(). Until a queue is wired, or if NULL
 * is wired, send_telemetry() is a safe no-op (nothing to post to).
 * _Requirements: 5.1, 5.2, 5.5, 5.6_
 */

/* Wire the destination telemetry queue. Called by obstacle_task_start()
 * (task 13.1) with Buddy 1's Telemetry_Queue handle; NULL disables posting. */
void obs_set_telemetry_queue(QueueHandle_t telemetry_queue);

/* Best-effort, non-blocking post of one obstacle telemetry message. profile is
 * packed with the given fault/flags byte into the 32-bit data lane. On a full
 * queue the message is dropped and FAULT_QUEUE_FULL is latched (no retry). */
void send_telemetry(const obstacle_profile_t *profile, uint8_t flags);

/* --------------------------------------------------------------------------
 * Task 10.1: coarse scan over the five fixed angles
 * --------------------------------------------------------------------------
 * coarse_scan() confirms or rejects a suspected obstacle with the fixed 5-point
 * sweep {30, 60, 90, 120, 150} deg (reqs 2.7, 1.9). It:
 *   1. posts a STOP to the Motor_Queue BEFORE any reading so the car is
 *      stationary throughout scanning (req 3.7, 2.7); a failed STOP post latches
 *      FAULT_QUEUE_FULL (req 3.9);
 *   2. resets the consecutive-NO_ECHO streak, then aim_and_read()s each of the
 *      exactly five angles in order, writing one filtered reading per angle into
 *      the caller-provided results[COARSE_SCAN_ANGLE_COUNT] array;
 *   3. feeds the streak into obs_update_no_echo_fault() each angle so three
 *      consecutive NO_ECHO timeouts latch FAULT_NO_ECHO and request the
 *      SCAN_FAULT transition (reqs 1.10, 5.7);
 *   4. applies the false-trigger check (req 1.9): if no filtered reading is both
 *      != NO_ECHO and < SAFE_LIMIT_CM, nothing persisted close, so the trigger
 *      was false.
 *
 * The result array is always fully written (five entries, each angle paired with
 * its filtered distance or OBS_DISTANCE_NO_ECHO) so the caller can inspect the
 * raw sweep regardless of the returned classification.
 *
 * Returns a coarse_scan_result_t telling the state machine (task 13.2) which
 * edge to take:
 *   COARSE_SCAN_FAULT     - three consecutive NO_ECHO latched FAULT_NO_ECHO.
 *   COARSE_FALSE_TRIGGER  - scan completed, nothing persisted below SAFE_LIMIT.
 *   COARSE_CONFIRMED      - at least one valid reading < SAFE_LIMIT_CM.
 *
 * A fault takes precedence over the confirm/false-trigger distinction: when the
 * NO_ECHO streak latches the fault, COARSE_SCAN_FAULT is returned even if an
 * earlier angle happened to see a close return.
 * _Requirements: 1.8, 1.9, 1.10, 2.7, 3.7, 5.7_
 */

/* Number of fixed coarse-sweep angles ({30,60,90,120,150}); sizes the caller's
 * results array. Kept in lockstep with COARSE_ANGLES in obstacle_integration.h. */
#define COARSE_SCAN_ANGLE_COUNT  5u

/* Outcome of a coarse scan - the edge the obstacle_task state machine takes. */
typedef enum {
    COARSE_CONFIRMED     = 0, /* a valid reading < SAFE_LIMIT_CM persisted       */
    COARSE_FALSE_TRIGGER = 1, /* nothing persisted below SAFE_LIMIT -> back idle */
    COARSE_SCAN_FAULT    = 2  /* 3 consecutive NO_ECHO latched FAULT_NO_ECHO     */
} coarse_scan_result_t;

/* Run the fixed 5-point coarse sweep, filling results[COARSE_SCAN_ANGLE_COUNT].
 * results must be non-NULL and have room for COARSE_SCAN_ANGLE_COUNT entries. */
coarse_scan_result_t coarse_scan(angle_reading_t *results);

/* --------------------------------------------------------------------------
 * Center-angle helper for the confirmed region (used by tasks 10.2 / 13.2)
 * --------------------------------------------------------------------------
 * Given a filled coarse-scan results array, return the servo angle of the
 * closest valid (non-NO_ECHO) reading - the center the fine scan windows around
 * (req 2.11 selection applied to the coarse sweep). On a tie the smallest angle
 * wins (deterministic). When the sweep holds no valid reading at all, returns 90
 * (the center of the servo range) as a safe default so a confirmed scan always
 * yields a usable center.
 */
uint8_t coarse_scan_center_angle(const angle_reading_t *results, uint8_t count);

/* --------------------------------------------------------------------------
 * Task 11.1: bypass execution + motor-queue plumbing
 * --------------------------------------------------------------------------
 * execute_bypass() posts the packed movement sequence for a chosen bypass side
 * to Buddy 2's Motor_Queue, in order (req 3.8). It builds a short bypass vector
 * (turn out, forward, turn back) whose direction depends on `cmd`
 * (MOTOR_CMD_BYPASS_LEFT vs MOTOR_CMD_BYPASS_RIGHT), packs each step with
 * obs_pack_motor_cmd() (14-bit distance_ticks payload), and posts each via
 * xQueueSend with a bounded block time <= 100 ms per attempt (req 3.4, 3.10).
 *
 * On a queue-full post it latches FAULT_QUEUE_FULL (req 3.9), retries within the
 * bounded block time, and - because every step of a bypass is a movement that
 * prevents a collision - escalates by posting a STOP when a step still fails
 * (req 3.10). The subsystem never writes the L298N pins GP2-GP7 directly
 * (req 3.5); all motion is a message on the Motor_Queue.
 *
 * The destination queue is stored by obstacle_task_start() (task 13.1) via the
 * internal setter obs_set_motor_queue(). Until a queue is wired, or if NULL is
 * wired, execute_bypass() is a safe no-op (nothing to post to, and no pins are
 * ever touched).
 * _Requirements: 3.4, 3.5, 3.8, 3.9, 3.10_
 */

/* Wire the destination motor command queue. Called by obstacle_task_start()
 * (task 13.1) with Buddy 2's Motor_Queue handle; NULL disables posting.
 *
 * SHARED PLUMBING: the coarse scan (task 10.1), bypass (this task 11.1), line
 * recovery (task 11.2), and the state machine (task 13.2) all post to the same
 * Motor_Queue. This single setter owns the file-static handle; siblings reuse
 * it rather than redeclaring their own. */
void obs_set_motor_queue(QueueHandle_t motor_queue);

/* Build and post the packed bypass movement sequence for `cmd` (a BYPASS_LEFT
 * or BYPASS_RIGHT decision from plan_bypass) to the Motor_Queue, in order. See
 * the block comment above for the queue-full retry / STOP-escalation policy. */
void execute_bypass(motor_cmd_t cmd);

/* --------------------------------------------------------------------------
 * Task 10.2: fine-scan profiling window
 * --------------------------------------------------------------------------
 * fine_scan() profiles a confirmed obstacle by sweeping a window of +/- 30 deg
 * around center_angle, clamped to the servo range [30, 150] (req 2.8), and
 * feeding the filtered readings to estimate_profile() (task 4.3) to produce the
 * obstacle_profile_t estimate (reqs 2.9-2.13). It:
 *   1. derives lo = clamp(center - 30, 30, 150) and hi = clamp(center + 30,
 *      30, 150) via servo_clamp_angle();
 *   2. sweeps lo..hi in ascending angle order with a step of FINE_SCAN_STEP_DEG
 *      (15 deg), taking exactly one filtered reading per angle with
 *      aim_and_read() (task 9.1) and ALWAYS including the hi endpoint so the far
 *      edge of the window is profiled;
 *   3. returns estimate_profile(readings, count, lo, hi).
 *
 * HONEST RESOLUTION BOUND (req 2.8, design): the step is NEVER finer than the
 * ~15 deg HC-SR04 beam width. A finer step adds no real angular resolution
 * (the beam physically overlaps), so the window deliberately steps at the beam
 * width and does not over-claim precision.
 *
 * Because the window spans at most 60 deg, at most five stepped angles plus the
 * hi endpoint are visited; the obstacle_profile_t is returned by value so no
 * caller-provided buffer is needed. center_angle is treated as an angle in the
 * servo span; out-of-range inputs are folded into the clamped [lo, hi] window.
 * _Requirements: 2.8, 2.9, 2.10, 2.11, 2.12, 2.13_
 */

/* Fine-scan angular step. Bounded to the ~15 deg beam width: stepping finer
 * than the beam adds no real resolution, so this is the floor (req 2.8). */
#define FINE_SCAN_STEP_DEG  15u

/* Run the +/- 30 deg fine-scan window around center_angle and return the
 * profile estimate (see the block comment above). */
obstacle_profile_t fine_scan(uint8_t center_angle);

/* --------------------------------------------------------------------------
 * Task 11.2: line reacquisition after a bypass (line_recovery)
 * --------------------------------------------------------------------------
 * line_recovery() runs the LINE_RECOVERY state (called by the state machine,
 * task 13.2, after execute_bypass). It reacquires Buddy 3's line so normal
 * line-following can resume:
 *   1. Poll Buddy 3's line_detected() every 50 ms (req 4.1).
 *   2. WHILE line_detected() == false: post exactly ONE packed search-movement
 *      command to the Motor_Queue per 50 ms cycle and stay in recovery rather
 *      than returning to idle (req 4.2). The per-cycle post is deliberately
 *      NON-escalating (a failed post latches FAULT_QUEUE_FULL and the search
 *      simply continues next cycle) so a transient full queue does not flip the
 *      car into a STOP mid-search.
 *   3. WHEN line_detected() == true: stop posting and return so the state
 *      machine hands control back to Buddy 3 and transitions to IDLE_MONITORING
 *      (req 4.3).
 *   4. IF the line is not reacquired within the 10 s recovery timeout measured
 *      from entry: post a STOP to the Motor_Queue and latch
 *      FAULT_RECOVERY_TIMEOUT (surfaced via obstacle_fault_flags()), then
 *      return (req 4.4).
 *
 * On ANY failed queue post (search-movement or the timeout STOP) FAULT_QUEUE_FULL
 * is latched and NO motor pin is ever written (req 4.5): all motion stays a
 * message on Buddy 2's single-writer Motor_Queue. The destination queue is the
 * SHARED file-static handle wired by obstacle_task_start() (task 13.1) via
 * obs_set_motor_queue(); until a queue is wired line_recovery() still never
 * touches a pin - it just latches FAULT_QUEUE_FULL.
 *
 * Timing is driven through the subsystem's delay seam (scan_delay_fn_t), so the
 * 50 ms poll cadence and the 10 s timeout are observable off-device with a
 * simulated clock (same instrumentation used by task 9.2).
 * _Requirements: 4.1, 4.2, 4.3, 4.4, 4.5_
 */
void line_recovery(void);

#endif /* OBSTACLE_SCAN_H */
