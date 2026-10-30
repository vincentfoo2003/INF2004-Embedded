/*
 * Ultrasonic Obstacle Profiling (Buddy 5 / Subsystem 5).
 *
 * This is the subsystem's main translation unit. It hosts the scan pipeline and
 * (in later tasks) the obstacle_task state machine. Tasks fill it in
 * incrementally:
 *   - task 9.1 : aim_and_read() + the per-angle sampling loop   (below)
 *   - tasks 10-13: coarse/fine scan, bypass, recovery, telemetry, state machine
 *
 * The public lifecycle functions (obstacle_setup / obstacle_task_start) are
 * still stubs here and are completed in task 13.1; the standalone demo main()
 * is finished in task 15.2. Pure logic lives in obstacle_logic.c and the
 * hardware drivers in servo.c / hcsr04.c, so this TU only orchestrates them.
 */

#include "pico/stdlib.h"      /* stdio_init_all (task 15.2 demo main) */

#include "obstacle_avoidance.h"
#include "obstacle_integration.h"
#include "obstacle_logic.h"   /* us_echo_to_cm, filter_samples (pure) */
#include "obstacle_scan.h"    /* aim_and_read + timing seams (this subsystem) */
#include "obstacle_faults.h"  /* obs_latch_fault + last-distance bookkeeping */
#include "servo.h"            /* servo_set_angle, servo_fault */
#include "hcsr04.h"           /* us_measure_echo_us */

#include "FreeRTOS.h"         /* FreeRTOS core types / config */
#include "task.h"             /* vTaskDelay, pdMS_TO_TICKS */
#include "queue.h"            /* QueueHandle_t, xQueueSend, pdPASS (task 12.1) */

/* Latest single-shot distance; starts as NO_ECHO until the first real reading. */
static uint8_t  s_last_distance_cm = OBS_DISTANCE_NO_ECHO;

/* Fault bitmask; 0 == healthy. */
static uint32_t s_fault_flags = 0u;

/* ==========================================================================
 * Task 9.1: aimed-and-filtered reading pipeline (aim_and_read)
 * --------------------------------------------------------------------------
 * One filtered distance per servo angle. See obstacle_scan.h for the full
 * contract and the rationale for the two swappable seams. Coarse/fine scans and
 * the obstacle_task state machine (tasks 10-13) build on top of this.
 * _Requirements: 1.6, 1.7, 2.2, 2.3, 2.4, 2.6, 5.3, 5.4_
 * ========================================================================== */

/* Consecutive angles that filtered to NO_ECHO; feeds the FAULT_NO_ECHO streak
 * (FAULT_NO_ECHO_STREAK == 3) checked by tasks 10.1 / 12.2. */
static uint8_t s_no_echo_streak = 0u;

/*
 * Default delay seam: sleep ms milliseconds on the real FreeRTOS scheduler.
 * pdMS_TO_TICKS converts to ticks for the configured tick rate. Isolated behind
 * a function pointer so a host test can substitute a simulated clock.
 */
static void scan_delay_ms_default(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* Active seams. Production build keeps the real HC-SR04 ranging cycle and the
 * real vTaskDelay; task 9.2's host test swaps them via the setters below. */
static us_measure_fn_t s_measure_hook = us_measure_echo_us;
static scan_delay_fn_t s_delay_hook   = scan_delay_ms_default;

void obs_set_measure_hook(us_measure_fn_t hook)
{
    s_measure_hook = (NULL == hook) ? us_measure_echo_us : hook;
}

void obs_set_delay_hook(scan_delay_fn_t hook)
{
    s_delay_hook = (NULL == hook) ? scan_delay_ms_default : hook;
}

uint8_t obs_no_echo_streak(void)
{
    return s_no_echo_streak;
}

void obs_reset_no_echo_streak(void)
{
    s_no_echo_streak = 0u;
}

uint8_t aim_and_read(uint8_t angle_deg)
{
    uint8_t samples[READINGS_PER_ANGLE];
    uint8_t i;
    uint8_t filtered;

    /* 1. Aim the servo. servo_set_angle() clamps to [30,150] internally and
     *    latches FAULT_SERVO itself on an out-of-range request. */
    servo_set_angle(angle_deg);

    /* 2. Let the servo physically settle before ranging at the new angle
     *    (req 2.6: fixed 200 ms, within the allowed 150..300 ms window). */
    s_delay_hook(SRV_SETTLE_MS);

    /* 3. Take exactly READINGS_PER_ANGLE (5) samples. Each sample is one full
     *    HC-SR04 ranging cycle converted to cm; a US_CYCLE_MS (>= 60 ms) delay
     *    AFTER each cycle keeps consecutive triggers >= 60 ms apart (req 5.3,
     *    5.4). The trailing delay also spaces this angle's last trigger from the
     *    next angle's first trigger. */
    for (i = 0u; i < (uint8_t)READINGS_PER_ANGLE; i++)
    {
        uint32_t echo_us = s_measure_hook();   /* 0 == NO_ECHO */

        samples[i] = us_echo_to_cm(echo_us);   /* 2..254, or 255 NO_ECHO */

        s_delay_hook(US_CYCLE_MS);             /* enforce the >= 60 ms floor */
    }

    /* 4. Reduce the 5 raw samples to one filtered value (mode when any value
     *    repeats, else median; all-NO_ECHO -> NO_ECHO). Pure, from task 3. */
    filtered = filter_samples(samples, (uint8_t)READINGS_PER_ANGLE);

    /* 5. Maintain the consecutive-NO_ECHO streak for fault detection: a valid
     *    reading clears it, an all-NO_ECHO angle extends it (saturating so the
     *    counter never wraps). */
    if (OBS_DISTANCE_NO_ECHO == filtered)
    {
        if (s_no_echo_streak < 0xFFu)
        {
            s_no_echo_streak++;
        }
    }
    else
    {
        s_no_echo_streak = 0u;
    }

    return filtered;
}

/* ==========================================================================
 * Task 13.1: subsystem lifecycle (obstacle_setup / obstacle_task_start)
 * --------------------------------------------------------------------------
 * obstacle_setup() brings up the hardware this subsystem owns exclusively: the
 * HC-SR04 TRIG/ECHO lines (GP16/GP17 via us_pins_init(), task 8.1) and the
 * GP12 servo PWM slice (servo_init(), task 7). servo_init() also clears the
 * FAULT_SERVO latch, so a fresh setup starts healthy (req 1.12, 2.5).
 *
 * obstacle_task_start() wires the two negotiated destination queues through the
 * subsystem's internal setters (obs_set_motor_queue / obs_set_telemetry_queue,
 * declared in obstacle_scan.h and defined by tasks 11.1 / 12.1) and creates the
 * single event-driven obstacle_task at the caller-chosen priority. The state
 * machine body (obstacle_task) is implemented by the NEXT wave, task 13.2;
 * it is forward-declared here so 13.1 compiles and can create the task, and
 * task 13.2 supplies the definition in this same TU.
 *
 * Task-creation conventions mirror the repo's existing xTaskCreate calls in
 * app/main.c: a configMINIMAL_STACK_SIZE stack and a named thread. The handle
 * is kept file-static so later tasks can reference the running task if needed.
 * _Requirements: 1.12, 2.5_
 * ========================================================================== */

/* Forward declaration of the state-machine task body. The definition is added
 * by task 13.2 in this same translation unit; declaring it here lets 13.1
 * create the task without pulling 13.2's implementation forward. */
static void obstacle_task(void *params);

/* Handle for the single obstacle_task. Kept file-static so the subsystem can
 * reference the running task later (e.g. diagnostics); NULL until created. */
static TaskHandle_t s_obstacle_task_handle = NULL;

/* ==========================================================================
 * Task 13.2: the event-driven obstacle_task state machine
 * --------------------------------------------------------------------------
 * The single schedulable body for this subsystem. It drives the design's
 * "Top-level task loop": a for(;;) over a switch(state) that walks
 *
 *   IDLE_MONITORING -> COARSE_SCAN -> FINE_SCAN_PROFILING -> AVOIDANCE_PLANNING
 *   -> EXECUTING_BYPASS -> LINE_RECOVERY -> IDLE_MONITORING
 *
 * with a SCAN_FAULT branch reachable from COARSE_SCAN. Each state calls the
 * building blocks assembled by tasks 9-12 (aim_and_read, coarse_scan, fine_scan,
 * plan_bypass, execute_bypass, line_recovery, send_telemetry) and the central
 * fault helpers (obstacle_fault_flags, obs_update_no_echo_fault), so this body
 * is pure orchestration - it owns no sensing/packing logic of its own.
 *
 * Hump gating (req 1.11): IDLE_MONITORING consults Buddy 4's terrain_hump_state()
 * FIRST. An incline/decline reads like a wall to a single HC-SR04, so while a
 * hump is active the proximity trigger is ignored (idle delay, then re-poll) and
 * the scan pipeline is never entered.
 *
 * Escalation to COARSE_SCAN (req 1.8): only when the hump is HUMP_NONE AND the
 * single-shot idle reading is a real echo (!= NO_ECHO) strictly inside
 * SAFE_LIMIT_CM. A NO_ECHO or far reading keeps the car idling.
 *
 * SCAN_FAULT (reqs 5.7, 5.8, 5.9): coarse_scan() has already latched
 * FAULT_NO_ECHO via obs_update_no_echo_fault() when three consecutive timeouts
 * occur; this state ensures the latch (idempotent), reports the fault through
 * telemetry (req 5.8), and - within this ONE loop iteration - posts an
 * EMERGENCY_STOP (a MOTOR_CMD_STOP packed command, reqs 3.4/3.5) when the car is
 * still moving (req 5.9), then returns to IDLE_MONITORING to recover per design.
 *
 * Movement bookkeeping: s_car_moving tracks whether the car is under motion so
 * the SCAN_FAULT EMERGENCY_STOP fires only when needed (req 5.9). The car is
 * treated as moving while line-following is active (IDLE_MONITORING) and while a
 * bypass / recovery is driving it; coarse_scan()'s pre-scan STOP makes it
 * stationary, so once a scan has begun the flag is cleared.
 * _Requirements: 1.8, 1.9, 1.10, 1.11, 3.7, 5.7, 5.8, 5.9_
 * ========================================================================== */

/* Marks the real obstacle_task body as defined. Task 13.1 left a placeholder
 * guarded by OBS_TASK_BODY_DEFINED ("TASK 13.2 REPLACES THIS"); that guarded
 * block is replaced by the definition below, and this #define is kept so any
 * stale guarded placeholder can never re-expand and clash with it. */
#define OBS_TASK_BODY_DEFINED

/* Forward declaration: the shared motor-post helper (STOP-escalating poster) is
 * defined further down in task 11.1's section. The SCAN_FAULT branch below uses
 * it for the EMERGENCY_STOP (req 5.9), so declare it here to keep this TU's
 * top-to-bottom compile order valid without moving the definition. */
static bool obs_post_motor_cmd_or_stop(uint16_t packed);

/* Idle-monitoring re-poll period. While nothing is suspected (or a hump is
 * active) the task sleeps this long between single-shot checks instead of
 * busy-spinning, keeping the >= 60 ms HC-SR04 cadence comfortably satisfied. */
#define OBS_IDLE_PERIOD_MS   100u

/* Explicit states for the obstacle_task loop (design "Top-level task loop"). */
typedef enum {
    OBS_STATE_IDLE_MONITORING     = 0,
    OBS_STATE_COARSE_SCAN         = 1,
    OBS_STATE_FINE_SCAN_PROFILING = 2,
    OBS_STATE_AVOIDANCE_PLANNING  = 3,
    OBS_STATE_EXECUTING_BYPASS    = 4,
    OBS_STATE_LINE_RECOVERY       = 5,
    OBS_STATE_SCAN_FAULT          = 6
} obstacle_state_t;

/* True while the car is believed to be under motion (line-following in idle, or
 * a bypass / recovery driving it). Cleared once a scan's pre-scan STOP has made
 * the car stationary. Drives the SCAN_FAULT EMERGENCY_STOP decision (req 5.9). */
static bool s_car_moving = true;

/*
 * The obstacle_task state machine. `params` is unused (the task owns no
 * per-instance context; all plumbing is file-static). Runs forever; it never
 * returns, matching the FreeRTOS task contract.
 */
static void obstacle_task(void *params)
{
    obstacle_state_t state = OBS_STATE_IDLE_MONITORING;

    /* Carried between states within one obstacle encounter: the coarse sweep
     * results, the fine-scan center, the resulting profile, and the planned
     * bypass command. Declared once here so the switch arms share them. */
    angle_reading_t    coarse_results[COARSE_SCAN_ANGLE_COUNT];
    uint8_t            center_angle = 90u;             /* servo-range center    */
    obstacle_profile_t profile;
    motor_cmd_t        bypass_cmd   = MOTOR_CMD_STOP;

    (void)params;

    for (;;)
    {
        switch (state)
        {
            case OBS_STATE_IDLE_MONITORING:
            {
                uint8_t d;

                /* Req 1.11: an active hump reads like a wall to the HC-SR04, so
                 * check Buddy 4 FIRST and ignore any proximity trigger while a
                 * hump is active - idle-delay and re-poll without ever entering
                 * the scan pipeline. */
                if (HUMP_NONE != terrain_hump_state())
                {
                    s_delay_hook(OBS_IDLE_PERIOD_MS);
                    break;   /* stay in IDLE_MONITORING */
                }

                /* Single-shot aimed reading at the range center. aim_and_read()
                 * handles the servo settle and the >= 60 ms cycle; the filtered
                 * result is recorded as the latest single-shot distance (req
                 * 5.6) for the public query helper. Line-following owns motion
                 * here, so the car is considered moving. */
                s_car_moving = true;
                d = aim_and_read(90u);
                obs_set_last_distance(d);

                /* Reqs 1.8 / 1.11: escalate to the coarse scan only on a real
                 * echo strictly inside the safe limit while the terrain is clear
                 * of a hump. The decision is the pure obs_should_escalate()
                 * guard (task 13.3) - the hump-first early-return above already
                 * handles the "ignore while hump active" idle-delay path, and
                 * passing the live terrain_hump_state() here keeps the guard
                 * total. A NO_ECHO or far reading keeps the car idling. */
                if (obs_should_escalate(d, terrain_hump_state()))
                {
                    state = OBS_STATE_COARSE_SCAN;
                }
                else
                {
                    s_delay_hook(OBS_IDLE_PERIOD_MS);
                }
                break;
            }

            case OBS_STATE_COARSE_SCAN:
            {
                coarse_scan_result_t result = coarse_scan(coarse_results);

                /* coarse_scan() posts a STOP before any reading (reqs 3.7, 2.7),
                 * so the car is stationary for the rest of the scan pipeline. */
                s_car_moving = false;

                if (COARSE_SCAN_FAULT == result)
                {
                    /* Three consecutive NO_ECHO latched FAULT_NO_ECHO (reqs
                     * 1.10, 5.7): the scan is unreliable -> fault branch. */
                    state = OBS_STATE_SCAN_FAULT;
                }
                else if (COARSE_CONFIRMED == result)
                {
                    /* Confirmed obstacle: profile around the closest return. */
                    center_angle = coarse_scan_center_angle(
                        coarse_results, (uint8_t)COARSE_SCAN_ANGLE_COUNT);
                    state = OBS_STATE_FINE_SCAN_PROFILING;
                }
                else
                {
                    /* COARSE_FALSE_TRIGGER (req 1.9): nothing persisted below
                     * the safe limit -> treat the trigger as false, back idle. */
                    state = OBS_STATE_IDLE_MONITORING;
                }
                break;
            }

            case OBS_STATE_FINE_SCAN_PROFILING:
            {
                /* Profile the confirmed region, then publish it with the current
                 * fault flags (best-effort, non-blocking). The car stays stopped
                 * throughout profiling (the coarse-scan STOP still holds). */
                profile = fine_scan(center_angle);
                send_telemetry(&profile,
                               (uint8_t)obstacle_fault_flags());

                if (!profile.present)
                {
                    /* Fine scan did not confirm a blocking obstacle -> idle. */
                    state = OBS_STATE_IDLE_MONITORING;
                }
                else
                {
                    state = OBS_STATE_AVOIDANCE_PLANNING;
                }
                break;
            }

            case OBS_STATE_AVOIDANCE_PLANNING:
            {
                /* Pick the bypass side with the greater estimated clearance. */
                bypass_cmd = plan_bypass(&profile);
                state = OBS_STATE_EXECUTING_BYPASS;
                break;
            }

            case OBS_STATE_EXECUTING_BYPASS:
            {
                /* Drive the bypass vector (turn out, forward, turn back) as
                 * packed Motor_Queue commands - the car is now moving. */
                s_car_moving = true;
                execute_bypass(bypass_cmd);
                state = OBS_STATE_LINE_RECOVERY;
                break;
            }

            case OBS_STATE_LINE_RECOVERY:
            {
                /* Reacquire Buddy 3's line (search movements until found or the
                 * recovery timeout). Still moving until the line is handed back. */
                s_car_moving = true;
                line_recovery();
                state = OBS_STATE_IDLE_MONITORING;
                break;
            }

            case OBS_STATE_SCAN_FAULT:
            {
                /* Ensure FAULT_NO_ECHO is latched (coarse_scan already latched it
                 * via obs_update_no_echo_fault(); this call is idempotent and
                 * re-asserts it if the streak has since reset). Req 5.7 / 1.10. */
                (void)obs_update_no_echo_fault();
                obs_latch_fault(FAULT_NO_ECHO);

                /* Req 5.8: report the fault through telemetry. There is no fresh
                 * profile for a scan fault, so publish a zeroed profile carrying
                 * the current fault flags (the flags byte is what matters here). */
                {
                    obstacle_profile_t fault_profile = {0};
                    send_telemetry(&fault_profile,
                                   (uint8_t)obstacle_fault_flags());
                }

                /* Req 5.9: within THIS single loop iteration, halt the car if it
                 * is still moving. The EMERGENCY_STOP is a MOTOR_CMD_STOP packed
                 * command on Buddy 2's Motor_Queue (reqs 3.4, 3.5) - the pins are
                 * never touched directly. obs_post_motor_cmd_or_stop() applies
                 * the bounded retry / STOP-escalation and latches FAULT_QUEUE_FULL
                 * on failure. */
                if (s_car_moving)
                {
                    (void)obs_post_motor_cmd_or_stop(
                        obs_pack_motor_cmd(MOTOR_CMD_STOP, 0u));
                    s_car_moving = false;
                }

                /* Per design, recover by returning to IDLE_MONITORING; the
                 * latched fault stays visible via obstacle_fault_flags() /
                 * telemetry until an explicit reset. */
                state = OBS_STATE_IDLE_MONITORING;
                break;
            }

            default:
            {
                /* Defensive: an unknown state collapses back to idle. */
                state = OBS_STATE_IDLE_MONITORING;
                break;
            }
        }
    }
}

void obstacle_setup(void)
{
    /* Bring up the HC-SR04 pins (GP16 TRIG out, GP17 ECHO in) and the GP12
     * servo PWM slice. servo_init() also clears the FAULT_SERVO latch. */
    us_pins_init();
    servo_init();
}

void obstacle_task_start(QueueHandle_t motor_cmd_queue,
                         QueueHandle_t telemetry_queue,
                         UBaseType_t   priority)
{
    /* Wire the negotiated destination queues through the shared internal
     * plumbing (tasks 11.1 / 12.1). NULL handles are accepted and simply make
     * the respective posters safe no-ops. */
    obs_set_motor_queue(motor_cmd_queue);
    obs_set_telemetry_queue(telemetry_queue);

    /* Create the single event-driven obstacle task at the requested priority.
     * Stack size / naming follow the repo convention (see app/main.c). The
     * task body (state machine) is provided by task 13.2. */
    (void)xTaskCreate(obstacle_task,
                      "ObstacleTask",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      priority,
                      &s_obstacle_task_handle);
}

uint8_t obstacle_last_distance_cm(void)
{
    return s_last_distance_cm;
}

uint32_t obstacle_fault_flags(void)
{
    /* Aggregate the latched subsystem faults with the servo driver's own live
     * latch: a servo clamp fault (req 3.11) surfaces here without the scan loop
     * having to poll the servo. The result is masked to the defined fault bits
     * so no stray high bit ever reaches the 8-bit telemetry flags lane. */
    uint32_t flags = s_fault_flags;

    if (servo_fault())
    {
        flags |= FAULT_SERVO;
    }

    return flags & FAULT_FLAGS_MASK;
}

/* ==========================================================================
 * Task 12.2: fault latching + last-distance bookkeeping
 * --------------------------------------------------------------------------
 * Central, sticky latch for the subsystem's own faults and a record of the
 * latest single-shot distance. The scan/bypass/recovery/telemetry paths (tasks
 * 10-13) latch through here; the public obstacle_fault_flags() /
 * obstacle_last_distance_cm() above read the results (fault_flags also ORs in
 * the servo driver's live latch).
 *
 * FAULT_SERVO is deliberately NOT stored in s_fault_flags - it is owned by the
 * servo driver and sourced live in obstacle_fault_flags(), so latching it here
 * would double-track one condition. obs_latch_fault() therefore masks the
 * incoming bits to the latchable set.
 * _Requirements: 1.10, 4.4, 5.6, 5.7, 5.8_
 * ========================================================================== */

/* Bits this subsystem latches itself. FAULT_SERVO is excluded (servo-owned). */
#define OBS_LATCHABLE_FAULTS \
    (FAULT_NO_ECHO | FAULT_QUEUE_FULL | FAULT_RECOVERY_TIMEOUT)

void obs_latch_fault(uint32_t flag)
{
    /* Sticky OR: once set, a bit stays set until obs_clear_faults(). Masking to
     * the latchable set keeps FAULT_SERVO (servo-owned) and any undefined bit
     * out of the stored mask. */
    s_fault_flags |= (flag & OBS_LATCHABLE_FAULTS);
}

bool obs_update_no_echo_fault(void)
{
    /* Req 5.7 / 1.10: three consecutive NO_ECHO timeouts latch FAULT_NO_ECHO
     * and signal the SCAN_FAULT transition. The streak itself is maintained by
     * aim_and_read(); here we only translate it into the latched fault. */
    if (obs_no_echo_streak() >= (uint8_t)FAULT_NO_ECHO_STREAK)
    {
        obs_latch_fault(FAULT_NO_ECHO);
        return true;
    }

    /* Already latched from an earlier streak? Stay in the fault condition even
     * if a later reading has since reset the streak counter. */
    return (0u != (s_fault_flags & FAULT_NO_ECHO));
}

bool obs_fault_is_set(uint32_t flag)
{
    /* Report against the aggregated view (latched + live servo fault) so the
     * state machine sees exactly what telemetry/consumers would see. */
    return (0u != (obstacle_fault_flags() & flag));
}

void obs_clear_faults(void)
{
    /* Clears only the latched subsystem faults; the servo driver's own latch is
     * reset by servo_init(). */
    s_fault_flags = 0u;
}

void obs_set_last_distance(uint8_t cm)
{
    /* Req 5.6: remember the most recent filtered reading for the public query
     * helper. Stored verbatim (2..254 or OBS_DISTANCE_NO_ECHO). */
    s_last_distance_cm = cm;
}

/* ==========================================================================
 * Task 12.1: non-blocking telemetry post (send_telemetry)
 * --------------------------------------------------------------------------
 * Publishes one obstacle telemetry message to Buddy 1's Telemetry_Queue
 * best-effort, without ever blocking the scan loop (the NFR12 relaxation:
 * sensing is governed by the >= 60 ms HC-SR04 cycle while telemetry is
 * decoupled and asynchronous). The message is a TelemetryData32 with
 * id == TELEMETRY_ID_OBSTACLE and the profile+flags bit-packed into the 32-bit
 * data lane by obs_pack_telemetry (task 2.4). The post uses a 0-tick try-post
 * (req 5.5); on a full queue the message is discarded and FAULT_QUEUE_FULL is
 * latched through the central latch (req 5.6) with NO synchronous retry.
 *
 * NOTE: the telemetry queue handle below is file-static plumbing wired by
 * obstacle_task_start() (task 13.1) via obs_set_telemetry_queue(). Keeping it
 * here (rather than in the public API) matches the motor-queue plumbing style
 * and keeps the destination out of the pure logic.
 * _Requirements: 5.1, 5.2, 5.5, 5.6_
 * ========================================================================== */

/* Destination telemetry queue; NULL until task 13.1 wires it (or if cleared).
 * A NULL queue makes send_telemetry() a safe no-op. */
static QueueHandle_t s_telemetry_queue = NULL;

void obs_set_telemetry_queue(QueueHandle_t telemetry_queue)
{
    s_telemetry_queue = telemetry_queue;
}

void send_telemetry(const obstacle_profile_t *profile, uint8_t flags)
{
    TelemetryData32 msg;

    /* Nothing to post to (queue not wired yet) or no profile to pack: skip.
     * This keeps the scan loop safe before task 13.1 installs the queue. */
    if ((NULL == s_telemetry_queue) || (NULL == profile))
    {
        return;
    }

    /* Build the obstacle message: a fixed subsystem id plus the profile and
     * fault/flags byte bit-packed into the 32-bit data lane. */
    msg.id   = TELEMETRY_ID_OBSTACLE;
    msg.data = obs_pack_telemetry(profile, flags);

    /* Non-blocking try-post: a 0-tick timeout returns immediately whether the
     * queue has room or not (req 5.5), so the scan loop is never stalled. */
    if (pdPASS != xQueueSend(s_telemetry_queue, &msg, (TickType_t)0))
    {
        /* Queue full: latch the fault, drop this message, and carry on. No
         * synchronous retry - telemetry is best-effort/decoupled (req 5.6). */
        obs_latch_fault(FAULT_QUEUE_FULL);
    }
}

/* ==========================================================================
 * Task 11.1: bypass execution via the Motor_Queue (execute_bypass)
 * --------------------------------------------------------------------------
 * Turns a chosen bypass side into a short sequence of packed movement commands
 * posted - in order (req 3.8) - to Buddy 2's single-writer Motor_Queue. The
 * subsystem never writes the L298N pins GP2-GP7 directly (req 3.5); all motion,
 * including the escalation STOP, is a message on the queue (req 3.4).
 *
 * Queue-full policy (req 3.9, 3.10): each post uses a bounded block time
 * (<= 100 ms per attempt). If a post still reports queue-full, FAULT_QUEUE_FULL
 * is latched and the post is retried within the bounded window. Because every
 * step of a bypass is a movement that prevents a collision, a step that cannot
 * be delivered escalates to a STOP post so the car halts rather than drifting
 * on a stale command.
 *
 * SHARED PLUMBING: the file-static motor-queue handle and its setter below are
 * shared by the coarse scan (task 10.1), this bypass (task 11.1), line recovery
 * (task 11.2), and the state machine (task 13.2). The block is guarded so that
 * whichever sibling TU edit lands first defines it once; later siblings REUSE
 * it rather than redefining s_motor_queue / obs_set_motor_queue.
 * _Requirements: 3.4, 3.5, 3.8, 3.9, 3.10_
 * ========================================================================== */

#ifndef OBS_MOTOR_QUEUE_PLUMBING_DEFINED
#define OBS_MOTOR_QUEUE_PLUMBING_DEFINED

/* Destination motor command queue; NULL until task 13.1 wires it (or if
 * cleared). A NULL queue makes every poster a safe no-op - importantly it still
 * means NO motor pin is ever touched directly (req 3.5). */
static QueueHandle_t s_motor_queue = NULL;

void obs_set_motor_queue(QueueHandle_t motor_queue)
{
    s_motor_queue = motor_queue;
}

#endif /* OBS_MOTOR_QUEUE_PLUMBING_DEFINED */

/* Per-attempt bounded block time for a motor post: the queue-full retry must
 * not exceed 100 ms per attempt (req 3.10). */
#define OBS_MOTOR_POST_BLOCK_MS   100u

/* How many bounded attempts a single step gets before it is treated as
 * undeliverable and escalated to a STOP. Two attempts keeps the worst-case
 * blocking bounded (<= 2 * 100 ms) while giving the single-writer consumer a
 * second chance to drain the queue. */
#define OBS_MOTOR_POST_ATTEMPTS   2u

/* Bypass vector payloads, expressed as 14-bit distance_ticks (encoder units,
 * 0..16383). ESTIMATES tuned at integration; kept well inside the 14-bit field.
 * The sequence is the classic "turn out, drive past, turn back". */
#define OBS_BYPASS_TURN_TICKS     200u  /* quarter-ish pivot away from the line */
#define OBS_BYPASS_FORWARD_TICKS  400u  /* straight leg clearing the obstacle   */

/*
 * Post one already-packed motor command to the Motor_Queue with the bounded
 * retry / STOP-escalation policy (req 3.9, 3.10).
 *
 * Every command in a bypass is a collision-preventing movement, so a step that
 * cannot be delivered within its bounded attempts escalates to a STOP: we post
 * MOTOR_CMD_STOP (also the EMERGENCY_STOP intent) so the car halts instead of
 * continuing on a stale command. The STOP is itself posted with the same
 * bounded block time; if even that cannot be delivered the fault stays latched
 * and we return - we never fall back to writing motor pins (req 3.5).
 *
 * Returns true when the intended command was accepted; false when it had to
 * escalate (or could not be delivered at all).
 */
static bool obs_post_motor_cmd_or_stop(uint16_t packed)
{
    TickType_t block = pdMS_TO_TICKS(OBS_MOTOR_POST_BLOCK_MS);
    uint8_t    attempt;
    uint16_t   stop_packed;

    /* No queue wired yet: nothing to post to. Latch the fault so the condition
     * is visible, and - crucially - never touch a motor pin (req 3.5). */
    if (NULL == s_motor_queue)
    {
        obs_latch_fault(FAULT_QUEUE_FULL);
        return false;
    }

    /* Bounded retry: each attempt blocks at most OBS_MOTOR_POST_BLOCK_MS
     * (<= 100 ms, req 3.10). A queue-full result latches FAULT_QUEUE_FULL
     * (req 3.9) and we try again until the attempt budget is spent. */
    for (attempt = 0u; attempt < OBS_MOTOR_POST_ATTEMPTS; attempt++)
    {
        if (pdPASS == xQueueSend(s_motor_queue, &packed, block))
        {
            return true;   /* delivered in order (req 3.8) */
        }

        obs_latch_fault(FAULT_QUEUE_FULL);
    }

    /* Still undeliverable after the bounded attempts. Because the failing
     * command is a movement required to prevent a collision, escalate by
     * posting a STOP (req 3.10). The STOP uses the same bounded block time and
     * latches the fault again on failure; we still never write the pins. */
    stop_packed = obs_pack_motor_cmd(MOTOR_CMD_STOP, 0u);
    if (pdPASS != xQueueSend(s_motor_queue, &stop_packed, block))
    {
        obs_latch_fault(FAULT_QUEUE_FULL);
    }

    return false;
}

void execute_bypass(motor_cmd_t cmd)
{
    /* The bypass is a short vector: turn OUT toward the open side, drive FORWARD
     * past the obstacle, then turn BACK toward the line. The turn-out side comes
     * from the planner's decision (cmd); the turn-back is the opposite side so
     * the car re-aims at the original heading. Non-bypass commands are ignored
     * defensively (the planner only ever yields BYPASS_LEFT / BYPASS_RIGHT). */
    motor_cmd_t turn_out;
    motor_cmd_t turn_back;
    uint16_t    seq[3];
    uint8_t     i;

    if (MOTOR_CMD_BYPASS_LEFT == cmd)
    {
        turn_out  = MOTOR_CMD_BYPASS_LEFT;
        turn_back = MOTOR_CMD_BYPASS_RIGHT;
    }
    else if (MOTOR_CMD_BYPASS_RIGHT == cmd)
    {
        turn_out  = MOTOR_CMD_BYPASS_RIGHT;
        turn_back = MOTOR_CMD_BYPASS_LEFT;
    }
    else
    {
        /* Not a bypass decision: nothing to execute. */
        return;
    }

    /* Pack each step with its 14-bit distance_ticks payload (req 3.4). The
     * order is fixed: turn out -> forward -> turn back (req 3.8). */
    seq[0] = obs_pack_motor_cmd(turn_out,          OBS_BYPASS_TURN_TICKS);
    seq[1] = obs_pack_motor_cmd(MOTOR_CMD_FORWARD, OBS_BYPASS_FORWARD_TICKS);
    seq[2] = obs_pack_motor_cmd(turn_back,         OBS_BYPASS_TURN_TICKS);

    /* Post each step in order. obs_post_motor_cmd_or_stop() handles the bounded
     * queue-full retry and the collision-preventing STOP escalation; once a step
     * has had to escalate to STOP, continuing the sequence would fight that STOP
     * with stale movement, so we stop driving the rest of the vector. */
    for (i = 0u; i < (uint8_t)(sizeof seq / sizeof seq[0]); i++)
    {
        if (!obs_post_motor_cmd_or_stop(seq[i]))
        {
            break;
        }
    }
}

/* ==========================================================================
 * Task 15.2: standalone demo / integration entry (demo main)
 * --------------------------------------------------------------------------
 * The obstacle_avoidance executable's own entry point. It stands up a minimal
 * FreeRTOS environment so the whole subsystem can be exercised on-device
 * WITHOUT the peer subsystems: it creates demo-local motor-command and
 * telemetry queues, brings the hardware up with obstacle_setup(), hands the two
 * queues to obstacle_task_start(), and starts the scheduler.
 *
 * This main() lives ONLY in this translation unit (the obstacle_avoidance Pico
 * target). It never touches app/main.c, so the wifi build (the `app` target) is
 * unaffected by design. As belt-and-suspenders the body is additionally wrapped
 * in OBSTACLE_NO_DEMO_MAIN so a build can compile it out explicitly if ever
 * needed. The host test build instead renames the symbol with
 * -Dmain=obstacle_avoidance_demo_main (see obstacle_test.c), which still works
 * here because the function is named `main` by default.
 *
 * Demo-local queues (reqs 5.1, 5.5):
 *   - The motor queue carries the packed uint16_t motor commands the subsystem
 *     posters send (xQueueSend(q, &packed, ...) where `packed` is a uint16_t
 *     built by obs_pack_motor_cmd()); its element size is therefore
 *     sizeof(uint16_t).
 *   - The telemetry queue carries TelemetryData32 messages exactly as
 *     send_telemetry() posts them (req 5.1), so its element size is
 *     sizeof(TelemetryData32) with the non-blocking try-post policy (req 5.5).
 * In the real system these are Buddy 2's Motor_Queue and Buddy 1's
 * Telemetry_Queue; here they are standalone so the demo runs on its own. A tiny
 * drain task consumes both so a long demo run never wedges on a full queue.
 * _Requirements: 5.1, 5.5_
 * ========================================================================== */

#ifndef OBSTACLE_NO_DEMO_MAIN

/* Depth of each demo-local queue. Shallow is fine: the drain task below empties
 * them continuously, so this only needs to absorb a short burst. */
#define OBS_DEMO_QUEUE_DEPTH   10u

/* Demo task priority for the obstacle task. Per the design the obstacle task
 * sits high (near safety), so it runs above the idle priority; this mirrors the
 * repo convention of tskIDLE_PRIORITY + N used in app/main.c. */
#define OBS_DEMO_TASK_PRIORITY   (tskIDLE_PRIORITY + 3u)

/*
 * Tiny consumer that keeps the demo-local queues drained so they never fill
 * during a standalone run. It owns both handles (passed verbatim as its task
 * parameter). Non-blocking receives with a short idle delay keep it off the CPU
 * between messages; the drained values are discarded (this is a demo sink, not
 * a real motor driver / telemetry sink).
 */
static void obs_demo_drain_task(void *params)
{
    QueueHandle_t *queues         = (QueueHandle_t *)params;
    QueueHandle_t  motor_queue    = (NULL != queues) ? queues[0] : NULL;
    QueueHandle_t  telemetry_queue = (NULL != queues) ? queues[1] : NULL;

    for (;;)
    {
        uint16_t        motor_cmd;
        TelemetryData32 telem;

        if (NULL != motor_queue)
        {
            while (pdPASS == xQueueReceive(motor_queue, &motor_cmd,
                                           (TickType_t)0))
            {
                /* demo sink: discard the drained motor command */
            }
        }

        if (NULL != telemetry_queue)
        {
            while (pdPASS == xQueueReceive(telemetry_queue, &telem,
                                           (TickType_t)0))
            {
                /* demo sink: discard the drained telemetry message */
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50u));
    }
}

int main(void)
{
    /* Handles for the two demo-local queues; kept static so the drain task can
     * reference them through its parameter for the life of the program. */
    static QueueHandle_t demo_queues[2] = { NULL, NULL };

    /* Bring up stdio (USB per this target's CMake) so the demo can be observed. */
    stdio_init_all();

    /* Motor-command queue: element size matches the packed uint16_t commands the
     * subsystem posters send (req 5.1). */
    demo_queues[0] = xQueueCreate((UBaseType_t)OBS_DEMO_QUEUE_DEPTH,
                                  sizeof(uint16_t));

    /* Telemetry queue: element size is sizeof(TelemetryData32), exactly what
     * send_telemetry() posts best-effort (reqs 5.1, 5.5). */
    demo_queues[1] = xQueueCreate((UBaseType_t)OBS_DEMO_QUEUE_DEPTH,
                                  sizeof(TelemetryData32));

    /* Configure the HC-SR04 pins and the servo PWM slice (req 1.12, 2.5). */
    obstacle_setup();

    /* Wire the demo-local queues into the subsystem and create the obstacle
     * task at a high-ish priority (near safety, per the design). */
    obstacle_task_start(demo_queues[0], demo_queues[1],
                        (UBaseType_t)OBS_DEMO_TASK_PRIORITY);

    /* Keep the demo-local queues drained so a standalone run never wedges. */
    (void)xTaskCreate(obs_demo_drain_task,
                      "ObstacleDemoDrain",
                      configMINIMAL_STACK_SIZE,
                      demo_queues,
                      tskIDLE_PRIORITY + 1u,
                      NULL);

    /* Hand control to FreeRTOS. vTaskStartScheduler() does not return while the
     * scheduler is running; the trailing loop is only reached if it ever fails
     * to start (e.g. out of heap), in which case we idle safely. */
    vTaskStartScheduler();

    for (;;)
    {
        /* scheduler failed to start: nothing left to do but idle */
    }

    return 0;
}

#endif /* OBSTACLE_NO_DEMO_MAIN */

/* ==========================================================================
 * Task 10.1: motor-queue seam + coarse scan over the five fixed angles
 * --------------------------------------------------------------------------
 * The coarse scan confirms or rejects a suspected obstacle with the fixed
 * 5-point sweep {30,60,90,120,150} deg (reqs 2.7, 1.9). Before any reading it
 * posts a STOP so the car is stationary throughout scanning (reqs 3.7, 2.7),
 * then takes exactly one filtered reading per angle via aim_and_read() (task
 * 9.1), feeds the consecutive-NO_ECHO streak into obs_update_no_echo_fault() so
 * three timeouts latch FAULT_NO_ECHO and request SCAN_FAULT (reqs 1.10, 5.7),
 * and finally applies the false-trigger check (req 1.9): a trigger is confirmed
 * only if some filtered reading is both != NO_ECHO and < SAFE_LIMIT_CM.
 *
 * Motor-queue seam: Buddy 5 never writes GP2-GP7; the pre-scan STOP is a packed
 * command posted to Buddy 2's Motor_Queue (reqs 3.4, 3.5). This reuses the
 * SHARED motor-queue plumbing defined above for task 11.1 - the file-static
 * s_motor_queue handle (wired by obstacle_task_start(), task 13.1, via
 * obs_set_motor_queue()) and the obs_post_motor_cmd_or_stop() poster, which
 * already applies the bounded queue-full retry and latches FAULT_QUEUE_FULL on
 * failure (req 3.9). No competing seam is introduced here.
 * _Requirements: 1.8, 1.9, 1.10, 2.7, 3.7, 3.9, 5.7_
 * ========================================================================== */

uint8_t coarse_scan_center_angle(const angle_reading_t *results, uint8_t count)
{
    uint8_t i;
    uint8_t best_angle    = 90u;                   /* safe default: range center */
    uint8_t best_distance = OBS_DISTANCE_NO_ECHO;  /* no valid reading seen yet   */

    if (NULL == results)
    {
        return best_angle;
    }

    /* Closest valid (non-NO_ECHO) reading wins; on a tie the first/smallest
     * angle is kept because we only replace on a strictly-smaller distance and
     * the array is swept in ascending-angle order (req 2.11, deterministic). */
    for (i = 0u; i < count; i++)
    {
        uint8_t d = results[i].distance_cm;

        if (OBS_DISTANCE_NO_ECHO == d)
        {
            continue;
        }

        if ((OBS_DISTANCE_NO_ECHO == best_distance) || (d < best_distance))
        {
            best_distance = d;
            best_angle    = results[i].angle_deg;
        }
    }

    return best_angle;
}

coarse_scan_result_t coarse_scan(angle_reading_t *results)
{
    static const uint8_t coarse_angles[COARSE_SCAN_ANGLE_COUNT] = COARSE_ANGLES;

    uint8_t i;
    bool    scan_fault   = false; /* three consecutive NO_ECHO latched the fault */
    bool    confirmed    = false; /* some reading persisted below SAFE_LIMIT_CM  */

    /* Defensive: a NULL results array cannot hold the sweep, so report a false
     * trigger (nothing confirmed) rather than writing through a bad pointer. */
    if (NULL == results)
    {
        return COARSE_FALSE_TRIGGER;
    }

    /* 1. Stop the car BEFORE any reading so it is stationary throughout the
     *    scan (reqs 3.7, 2.7). Posted through the shared poster (task 11.1),
     *    which applies the bounded queue-full retry and latches FAULT_QUEUE_FULL
     *    itself on failure (req 3.9). We still proceed to scan on failure: the
     *    fault surfaces via telemetry / obstacle_fault_flags(), and for a STOP
     *    the poster's own STOP-escalation is simply idempotent. */
    (void)obs_post_motor_cmd_or_stop(obs_pack_motor_cmd(MOTOR_CMD_STOP, 0u));

    /* 2. Fresh streak for this scan so a NO_ECHO run from a previous scan does
     *    not bleed into this one's 3-consecutive-timeout fault decision. */
    obs_reset_no_echo_streak();

    /* 3. Sweep EXACTLY the five fixed angles in order, one filtered reading per
     *    angle (req 2.7). Each aim_and_read() updates the consecutive-NO_ECHO
     *    streak; obs_update_no_echo_fault() latches FAULT_NO_ECHO once the
     *    streak reaches FAULT_NO_ECHO_STREAK (3) timeouts (reqs 1.10, 5.7). */
    for (i = 0u; i < (uint8_t)COARSE_SCAN_ANGLE_COUNT; i++)
    {
        uint8_t angle = coarse_angles[i];
        uint8_t d     = aim_and_read(angle);

        results[i].angle_deg   = angle;
        results[i].distance_cm = d;

        /* Record the latest single-shot distance for the public query helper
         * (req 5.6), mirroring the idle-monitoring path. */
        obs_set_last_distance(d);

        /* Streak -> fault. Once latched it stays latched for this scan, so a
         * later valid reading cannot clear the SCAN_FAULT decision. */
        if (obs_update_no_echo_fault())
        {
            scan_fault = true;
        }

        /* 4. False-trigger check (req 1.9): a trigger is confirmed only when a
         *    filtered reading is both a real echo and strictly inside the
         *    safe limit. Scan the full sweep so one close angle anywhere
         *    confirms. */
        if ((OBS_DISTANCE_NO_ECHO != d) && (d < (uint8_t)SAFE_LIMIT_CM))
        {
            confirmed = true;
        }
    }

    /* A latched NO_ECHO fault takes precedence: the scan is unreliable, so the
     * state machine must go to SCAN_FAULT even if an early angle saw a close
     * return (reqs 1.10, 5.7). */
    if (scan_fault)
    {
        return COARSE_SCAN_FAULT;
    }

    /* Nothing persisted below SAFE_LIMIT_CM across the five angles -> treat the
     * original proximity trigger as false and return to idle (req 1.9). */
    return confirmed ? COARSE_CONFIRMED : COARSE_FALSE_TRIGGER;
}

/* ==========================================================================
 * Task 10.2: fine-scan profiling window (fine_scan)
 * --------------------------------------------------------------------------
 * Profiles a confirmed obstacle by sweeping a +/- 30 deg window around the
 * center angle chosen by the coarse scan, then reduces the filtered readings to
 * one obstacle_profile_t ESTIMATE via estimate_profile() (task 4.3).
 *
 * Window (req 2.8): lo = clamp(center - 30, 30, 150), hi = clamp(center + 30,
 * 30, 150), both derived with the pure servo_clamp_angle() helper (task 7.1) so
 * the window always lies inside the servo's mechanical range. The sweep walks
 * lo..hi in ascending angle order at FINE_SCAN_STEP_DEG (15 deg) steps and
 * always finishes on the hi endpoint so the far edge of the window is profiled
 * even when (hi - lo) is not an exact multiple of the step.
 *
 * HONEST RESOLUTION BOUND (req 2.8 / design): the step is never finer than the
 * ~15 deg beam width. A single HC-SR04 beam physically overlaps across smaller
 * steps, so a finer step would add no real angular resolution - only more
 * samples for the modal filter. The design explicitly does NOT over-claim
 * sub-beam precision, so 15 deg is the floor.
 *
 * Each angle contributes exactly one filtered reading via aim_and_read()
 * (task 9.1), which also keeps the >= 60 ms cycle floor and the servo settle.
 * The collected readings are handed to estimate_profile() with the same
 * [lo, hi] window, and the resulting profile (reqs 2.9-2.13) is returned by
 * value. No caller buffer and no motor-queue traffic are involved here - the
 * STOP that holds the car stationary for profiling was already posted by the
 * coarse scan (task 10.1) / the state machine (task 13.2).
 * _Requirements: 2.8, 2.9, 2.10, 2.11, 2.12, 2.13_
 * ========================================================================== */

/* Window half-width: the fine scan sweeps +/- this many degrees around center
 * before clamping to the servo range (req 2.8). */
#define FINE_SCAN_HALF_WINDOW_DEG  30u

/* Upper bound on stepped angles in a +/- 30 deg (<= 60 deg wide) window at a
 * 15 deg step: floor(60 / 15) + 1 == 5 stepped angles, plus one slot for the
 * hi endpoint when it does not fall on a step boundary. */
#define FINE_SCAN_MAX_READINGS \
    (((2u * FINE_SCAN_HALF_WINDOW_DEG) / FINE_SCAN_STEP_DEG) + 2u)

obstacle_profile_t fine_scan(uint8_t center_angle)
{
    angle_reading_t readings[FINE_SCAN_MAX_READINGS];
    uint8_t         count = 0u;
    uint8_t         lo;
    uint8_t         hi;
    uint8_t         angle;

    /* 1. Build the clamped +/- 30 deg window. servo_clamp_angle() folds any
     *    out-of-range endpoint onto the nearest servo bound, so lo/hi always
     *    lie inside [SRV_MIN_DEG, SRV_MAX_DEG] (req 2.8). Guard the center-30
     *    subtraction against unsigned underflow before clamping. */
    {
        uint8_t raw_lo = (center_angle > FINE_SCAN_HALF_WINDOW_DEG)
                         ? (uint8_t)(center_angle - FINE_SCAN_HALF_WINDOW_DEG)
                         : 0u;
        uint8_t raw_hi = (center_angle <= (uint8_t)(0xFFu - FINE_SCAN_HALF_WINDOW_DEG))
                         ? (uint8_t)(center_angle + FINE_SCAN_HALF_WINDOW_DEG)
                         : 0xFFu;

        lo = servo_clamp_angle(raw_lo, NULL);
        hi = servo_clamp_angle(raw_hi, NULL);
    }

    /* 2. Sweep lo..hi in ascending order at the beam-width step (req 2.8). One
     *    filtered reading per angle via aim_and_read() (task 9.1). The loop
     *    stops one step short of overshooting hi; the hi endpoint is appended
     *    separately below so the far window edge is always profiled. */
    for (angle = lo;
         (angle < hi) && (count < (uint8_t)FINE_SCAN_MAX_READINGS);
         angle = (uint8_t)(angle + (uint8_t)FINE_SCAN_STEP_DEG))
    {
        readings[count].angle_deg   = angle;
        readings[count].distance_cm = aim_and_read(angle);
        count++;
    }

    /* 3. Always profile the hi endpoint. Skip only if the last stepped angle
     *    already landed exactly on hi (a degenerate zero-width window where
     *    lo == hi still records that single angle here). */
    if ((count < (uint8_t)FINE_SCAN_MAX_READINGS) &&
        ((0u == count) || (readings[count - 1u].angle_deg != hi)))
    {
        readings[count].angle_deg   = hi;
        readings[count].distance_cm = aim_and_read(hi);
        count++;
    }

    /* 4. Reduce the window to one profile ESTIMATE over the same [lo, hi] span
     *    (reqs 2.9-2.13). estimate_profile() ignores NO_ECHO readings and marks
     *    every field as an estimate. */
    return estimate_profile(readings, count, lo, hi);
}

/* ==========================================================================
 * Task 11.2: line reacquisition after a bypass (line_recovery)
 * --------------------------------------------------------------------------
 * After execute_bypass() has steered the car around a confirmed obstacle, the
 * car is off the original line and Buddy 3's line-following is paused. This
 * function drives a short search routine and polls Buddy 3's line_detected()
 * until the line is reacquired, then hands control back:
 *
 *   - Poll line_detected() every 50 ms (req 4.1).
 *   - WHILE line_detected() == false: post exactly ONE packed search-movement
 *     command per 50 ms cycle and stay in recovery (req 4.2). The search step
 *     is a small FORWARD nudge (a packed Motor_Queue command, never a motor
 *     pin - req 4.5) that creeps the car forward to re-cross the line.
 *   - WHEN line_detected() == true: stop posting and return so the state
 *     machine transitions back to IDLE_MONITORING and Buddy 3 resumes (req 4.3).
 *   - On the 10 s recovery timeout (measured from entry): post a STOP and latch
 *     FAULT_RECOVERY_TIMEOUT so obstacle_fault_flags() surfaces it for the
 *     Safety / Vehicle Controller (req 4.4), then return.
 *
 * Queue-full policy (req 4.5): the per-cycle search post is deliberately
 * NON-escalating - a failed post latches FAULT_QUEUE_FULL and the loop simply
 * tries again on the next 50 ms cycle, so a transient full queue never flips the
 * car into a STOP mid-search. The terminal timeout STOP reuses the shared
 * poster obs_post_motor_cmd_or_stop() (its STOP-escalation is idempotent for a
 * STOP). In BOTH paths a failed post only latches FAULT_QUEUE_FULL; NO motor pin
 * is ever written - all motion stays a message on Buddy 2's single-writer
 * Motor_Queue (reqs 3.5, 4.5).
 *
 * SHARED PLUMBING: posts go through the file-static s_motor_queue handle wired
 * by obstacle_task_start() (task 13.1) via obs_set_motor_queue(), reusing the
 * same seam as the coarse scan (task 10.1) and bypass (task 11.1). Timing uses
 * the swappable s_delay_hook so the 50 ms cadence and 10 s timeout are testable
 * off-device with a simulated clock.
 * _Requirements: 4.1, 4.2, 4.3, 4.4, 4.5_
 * ========================================================================== */

/* Poll / search cadence: one line_detected() check and (while searching) one
 * search-movement post per 50 ms cycle (req 4.1, 4.2). */
#define OBS_RECOVERY_POLL_MS       50u

/* Recovery timeout window: give up reacquiring the line after 10 s measured
 * from entering recovery (req 4.4). Tracked in elapsed milliseconds so the
 * timeout is independent of the tick rate. */
#define OBS_RECOVERY_TIMEOUT_MS    10000u

/* Search-movement payload: a small FORWARD creep (14-bit distance_ticks, well
 * inside the 0..16383 field) so each cycle nudges the car forward to re-cross
 * the line rather than lunging. ESTIMATE tuned at integration. */
#define OBS_RECOVERY_SEARCH_TICKS  60u

/*
 * Post one per-cycle search-movement command WITHOUT the STOP escalation used
 * by the bypass poster: a single bounded xQueueSend, and on failure latch
 * FAULT_QUEUE_FULL and return (the loop retries next cycle). A NULL queue is a
 * safe no-op that still latches the fault and - crucially - never touches a
 * motor pin (reqs 4.5, 3.5).
 */
static void obs_post_search_move(uint16_t packed)
{
    TickType_t block = pdMS_TO_TICKS(OBS_MOTOR_POST_BLOCK_MS);

    if (NULL == s_motor_queue)
    {
        obs_latch_fault(FAULT_QUEUE_FULL);
        return;
    }

    if (pdPASS != xQueueSend(s_motor_queue, &packed, block))
    {
        /* Queue full this cycle: latch the fault, drop the search step, and let
         * the next 50 ms cycle try again. No escalation to STOP and no pin
         * write (req 4.5). */
        obs_latch_fault(FAULT_QUEUE_FULL);
    }
}

void line_recovery(void)
{
    uint32_t elapsed_ms = 0u;
    uint16_t search_packed;

    /* Pre-pack the per-cycle search-movement command once (a small FORWARD
     * creep). All motion is a packed Motor_Queue message - never a motor pin. */
    search_packed = obs_pack_motor_cmd(MOTOR_CMD_FORWARD,
                                       (uint16_t)OBS_RECOVERY_SEARCH_TICKS);

    /* Poll every 50 ms (req 4.1). While the line is not yet found, post exactly
     * one search-movement command per cycle and stay in recovery (req 4.2). */
    while (!line_detected())
    {
        /* 10 s timeout measured from entry (req 4.4): post STOP, latch the
         * recovery fault (surfaced by obstacle_fault_flags()), and return. The
         * STOP goes through the shared poster, which latches FAULT_QUEUE_FULL on
         * its own failure and never writes a pin (req 4.5). */
        if (elapsed_ms >= (uint32_t)OBS_RECOVERY_TIMEOUT_MS)
        {
            (void)obs_post_motor_cmd_or_stop(
                obs_pack_motor_cmd(MOTOR_CMD_STOP, 0u));
            obs_latch_fault(FAULT_RECOVERY_TIMEOUT);
            return;
        }

        /* One search-movement command this cycle (req 4.2). Non-escalating:
         * a failed post only latches FAULT_QUEUE_FULL (req 4.5). */
        obs_post_search_move(search_packed);

        /* Wait out the 50 ms poll cycle, then account the elapsed time toward
         * the recovery timeout. */
        s_delay_hook(OBS_RECOVERY_POLL_MS);
        elapsed_ms += (uint32_t)OBS_RECOVERY_POLL_MS;
    }

    /* line_detected() == true: stop posting search commands and return so the
     * state machine hands control back to Buddy 3 and goes to IDLE_MONITORING
     * (req 4.3). No further motor command is posted here - Buddy 3 resumes. */
}
