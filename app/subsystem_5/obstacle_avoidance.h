#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

/*
 * Ultrasonic Obstacle Profiling (Buddy 5 / Subsystem 5) - public API.
 *
 * This header exposes the subsystem's lifecycle and query functions to the
 * rest of the system. The implementation lives in the subsystem's .c files and
 * is built incrementally by the tasks in the spec's tasks.md.
 *
 * Conventions: Barr-C style - snake_case functions, #define macros, header
 * guards, one translation unit per driver.
 */

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

/* --------------------------------------------------------------------------
 * Distance encoding
 * --------------------------------------------------------------------------
 * Distances are stored as uint8_t (minimum-bit data constraint). Valid
 * readings are clamped to OBS_DISTANCE_MAX_CM (254). The top value 255 is
 * reserved as OBS_DISTANCE_NO_ECHO and is NEVER a valid measurement, so a
 * timeout is always distinguishable from a real reading.
 */

/* Reserved sentinel: NOT a valid distance. Distinguishes a no-echo/timeout
 * result from a genuine reading. Chosen as the top of the uint8_t range. */
#define OBS_DISTANCE_NO_ECHO   ((uint8_t)255u)
#define OBS_DISTANCE_MAX_CM    ((uint8_t)254u)   /* clamp ceiling for valid readings */

/* --------------------------------------------------------------------------
 * Fault flags (bitmask exposed via obstacle_fault_flags()); 0 == healthy.
 * -------------------------------------------------------------------------- */
#define FAULT_NO_ECHO           (1u << 0)  /* repeated ECHO timeouts */
#define FAULT_SERVO             (1u << 1)  /* servo command out of range / init fail */
#define FAULT_QUEUE_FULL        (1u << 2)  /* motor or telemetry queue post failed */
#define FAULT_RECOVERY_TIMEOUT  (1u << 3)  /* line_detected() not seen within the
                                            * recovery timeout (req 4.4) */

/* All fault bits live in the low nibble so the whole bitmask fits the 8-bit
 * telemetry flags lane [31:24] (design.md) with room to spare. */
#define FAULT_FLAGS_MASK  (FAULT_NO_ECHO | FAULT_SERVO | FAULT_QUEUE_FULL | \
                           FAULT_RECOVERY_TIMEOUT)

/* --------------------------------------------------------------------------
 * Public lifecycle
 * -------------------------------------------------------------------------- */

/* Configure GP16 TRIG (out), GP17 ECHO (in), GP12 servo PWM (50 Hz slice). */
void obstacle_setup(void);

/* Create the event-driven obstacle task. The caller passes the queues this
 * subsystem must publish to (negotiated interfaces, see obstacle_integration.h). */
void obstacle_task_start(QueueHandle_t motor_cmd_queue,
                         QueueHandle_t telemetry_queue,
                         UBaseType_t   priority);

/* --------------------------------------------------------------------------
 * Public query helpers (safe to call from other tasks)
 * -------------------------------------------------------------------------- */

/* Latest single-shot distance in cm, or OBS_DISTANCE_NO_ECHO. */
uint8_t obstacle_last_distance_cm(void);

/* Current fault bitmask (see FAULT_* flags). 0 == healthy. */
uint32_t obstacle_fault_flags(void);

#endif /* OBSTACLE_AVOIDANCE_H */
