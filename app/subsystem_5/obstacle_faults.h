#ifndef OBSTACLE_FAULTS_H
#define OBSTACLE_FAULTS_H

/*
 * Ultrasonic Obstacle Profiling (Buddy 5 / Subsystem 5) - fault/state latch
 * (INTERNAL header).
 *
 * This header exposes the subsystem's fault-latching and last-distance book-
 * keeping to the rest of the subsystem (the coarse/fine scans, bypass, line
 * recovery, telemetry, and the obstacle_task state machine in tasks 10-13). It
 * is NOT part of the public subsystem API in obstacle_avoidance.h - the only
 * externally published query surface stays obstacle_fault_flags() /
 * obstacle_last_distance_cm() there.
 *
 * Why a central latch (task 12.2):
 *   - Fault bits are "sticky": once a condition is detected (3 consecutive
 *     NO_ECHO timeouts, a queue-full post, a servo clamp, a line-recovery
 *     timeout) the corresponding FAULT_* bit stays set so the next telemetry
 *     message and the Safety / Vehicle Controller can surface it (reqs 5.6,
 *     5.7, 5.8, 4.4). Latching through one place keeps that policy consistent
 *     across every caller.
 *   - FAULT_SERVO is NOT latched here: the servo driver owns its own latch
 *     (servo_fault()); obstacle_fault_flags() ORs it in live so a clamp fault
 *     surfaces without the scan loop having to poll the servo (req 3.11).
 *
 * Conventions: Barr-C style - snake_case functions, #define macros, header
 * guards.
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_avoidance.h"     /* FAULT_* bits, OBS_DISTANCE_NO_ECHO */

/* --------------------------------------------------------------------------
 * Fault latch (shared by tasks 10.1 / 11.1 / 11.2 / 12.1 / 13.2)
 * --------------------------------------------------------------------------
 * The latched bitmask is the subsystem's own faults (NO_ECHO, QUEUE_FULL,
 * RECOVERY_TIMEOUT). FAULT_SERVO is sourced live from the servo driver by
 * obstacle_fault_flags(), so it must not be passed to obs_latch_fault().
 */

/* OR one or more FAULT_* bits into the latched fault mask. The bits are sticky
 * (latched) until obs_clear_faults(). Callers:
 *   - coarse/fine scan      -> FAULT_NO_ECHO (via obs_update_no_echo_fault)
 *   - bypass / recovery      -> FAULT_QUEUE_FULL on a failed queue post
 *   - line_recovery (11.2)   -> FAULT_RECOVERY_TIMEOUT on the 10 s timeout
 *   - telemetry post (12.1)  -> FAULT_QUEUE_FULL on a full telemetry queue
 * Bits outside FAULT_FLAGS_MASK are ignored so the latch can never hold a bit
 * that has no telemetry lane. */
void obs_latch_fault(uint32_t flag);

/* Latch FAULT_NO_ECHO when (and only when) the consecutive-NO_ECHO streak has
 * reached FAULT_NO_ECHO_STREAK (3) timeouts; req 5.7 / 1.10. The scan loop
 * calls this once per angle after aim_and_read() has updated the streak.
 * Returns true when the fault is latched (i.e. the SCAN_FAULT transition should
 * be taken), false otherwise. Idempotent: calling it again while the fault is
 * already latched simply returns true. */
bool obs_update_no_echo_fault(void);

/* True when the given FAULT_* bit is currently set in the aggregated fault
 * view (the latched faults ORed with the live servo fault). Convenience for the
 * state machine (e.g. "is an unrecoverable scan fault active?"). */
bool obs_fault_is_set(uint32_t flag);

/* Clear all latched faults back to healthy (0). Does NOT clear the servo
 * driver's own latch (that is reset by servo_init()). Intended for a future
 * explicit recovery/reset path; the scan loop never needs it mid-operation. */
void obs_clear_faults(void);

/* --------------------------------------------------------------------------
 * Latest single-shot distance (req 5.6 bookkeeping)
 * --------------------------------------------------------------------------
 * The sampling pipeline records every filtered reading here so the public
 * obstacle_last_distance_cm() always reflects the most recent measurement
 * (OBS_DISTANCE_NO_ECHO until the first real reading, or on a timeout).
 */

/* Store the most recent filtered single-shot distance (2..254, or
 * OBS_DISTANCE_NO_ECHO). Published verbatim by obstacle_last_distance_cm(). */
void obs_set_last_distance(uint8_t cm);

#endif /* OBSTACLE_FAULTS_H */
