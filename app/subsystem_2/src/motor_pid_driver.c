#include "motor_pid_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "task.h"

/* Fixed-Point PID Gains */
#define KP_SCALED               (120)
#define KI_SCALED               (5)
#define KD_SCALED               (20)
#define SCALE_SHIFT             (4U)

/* Weak default definitions allow standalone test compilation or full integration */
__attribute__((weak)) volatile uint8_t g_system_faults = 0U;
__attribute__((weak)) QueueHandle_t    g_motor_queue   = NULL;

__attribute__((weak)) void barcode_ir_isr_handler(void)
{
}

__attribute__((weak)) void imu_shock_isr_handler(void)
{
}

__attribute__((weak)) bool mqtt_enqueue_telemetry(uint8_t source_id, uint32_t packed_data)
{
    (void)source_id;
    (void)packed_data;
    return true;
}

static volatile int32_t g_left_ticks  = 0;
static volatile int32_t g_right_ticks = 0;
static motor_config_t   g_motor_cfg   = {PWM_WRAP_VALUE, MAX_SAFE_SPEED_PWM, false};

void master_gpio_callback(uint gpio, uint32_t events)
{
    (void)events;

    switch (gpio)
    {
        case PIN_ENC_L_A:
        {
            if (gpio_get(PIN_ENC_L_B))
            {
                g_left_ticks--;
            }
            else
            {
                g_left_ticks++;
            }
            break;
        }

        case PIN_ENC_R_A:
        {
            if (gpio_get(PIN_ENC_R_B))
            {
                g_right_ticks++;
            }
            else
            {
                g_right_ticks--;
            }
            break;
        }

        case PIN_IR_R_BARCODE:
        {
            barcode_ir_isr_handler();
            break;
        }

        case PIN_IMU_INT:
        {
            imu_shock_isr_handler();
            break;
        }

        default:
        {
            break;
        }
    }
}

motor_config_t motor_setup(uint16_t pwm_wrap, uint16_t max_speed, bool log)
{
    g_motor_cfg.pwm_wrap  = (0U == pwm_wrap) ? PWM_WRAP_VALUE : pwm_wrap;
    g_motor_cfg.max_speed = (0U == max_speed) ? MAX_SAFE_SPEED_PWM : max_speed;
    g_motor_cfg.log       = log;

    const uint8_t motor_pins[4] = {
        PIN_MOT_L_M1A,
        PIN_MOT_L_M1B,
        PIN_MOT_R_M2A,
        PIN_MOT_R_M2B
    };

    for (uint8_t idx = 0U; idx < 4U; idx++)
    {
        gpio_set_function(motor_pins[idx], GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(motor_pins[idx]);
        pwm_set_wrap(slice_num, g_motor_cfg.pwm_wrap);
        pwm_set_gpio_level(motor_pins[idx], 0U);
        pwm_set_enabled(slice_num, true);
    }

    const uint8_t enc_pins[4] = {
        PIN_ENC_L_A,
        PIN_ENC_L_B,
        PIN_ENC_R_A,
        PIN_ENC_R_B
    };

    for (uint8_t idx = 0U; idx < 4U; idx++)
    {
        gpio_init(enc_pins[idx]);
        gpio_set_dir(enc_pins[idx], GPIO_IN);
        gpio_pull_up(enc_pins[idx]);
    }

    gpio_set_irq_enabled_with_callback(PIN_ENC_L_A, GPIO_IRQ_EDGE_RISE, true, &master_gpio_callback);
    gpio_set_irq_enabled(PIN_ENC_R_A, GPIO_IRQ_EDGE_RISE, true);

    if (g_motor_cfg.log)
    {
        printf("[MOTOR] Setup complete (wrap=%u, max_speed=%u)\n",
               g_motor_cfg.pwm_wrap, g_motor_cfg.max_speed);
    }

    return g_motor_cfg;
}

void motor_set_raw(uint8_t cmd_mode, uint16_t pwm_left, uint16_t pwm_right)
{
    if (pwm_left > g_motor_cfg.pwm_wrap)
    {
        pwm_left = g_motor_cfg.pwm_wrap;
    }

    if (pwm_right > g_motor_cfg.pwm_wrap)
    {
        pwm_right = g_motor_cfg.pwm_wrap;
    }

    if ((0U == pwm_left) && (0U == pwm_right))
    {
        pwm_set_gpio_level(PIN_MOT_L_M1A, 0U);
        pwm_set_gpio_level(PIN_MOT_L_M1B, 0U);
        pwm_set_gpio_level(PIN_MOT_R_M2A, 0U);
        pwm_set_gpio_level(PIN_MOT_R_M2B, 0U);
    }
    else
    {
        switch (cmd_mode)
        {
            case MOT_CMD_FORWARD:
            {
                pwm_set_gpio_level(PIN_MOT_L_M1A, pwm_left);
                pwm_set_gpio_level(PIN_MOT_L_M1B, 0U);
                pwm_set_gpio_level(PIN_MOT_R_M2A, pwm_right);
                pwm_set_gpio_level(PIN_MOT_R_M2B, 0U);
                break;
            }

            case MOT_CMD_LEFT:
            {
                pwm_set_gpio_level(PIN_MOT_L_M1A, 0U);
                pwm_set_gpio_level(PIN_MOT_L_M1B, pwm_left);
                pwm_set_gpio_level(PIN_MOT_R_M2A, pwm_right);
                pwm_set_gpio_level(PIN_MOT_R_M2B, 0U);
                break;
            }

            case MOT_CMD_RIGHT:
            case MOT_CMD_UTURN:
            default:
            {
                pwm_set_gpio_level(PIN_MOT_L_M1A, pwm_left);
                pwm_set_gpio_level(PIN_MOT_L_M1B, 0U);
                pwm_set_gpio_level(PIN_MOT_R_M2A, 0U);
                pwm_set_gpio_level(PIN_MOT_R_M2B, pwm_right);
                break;
            }
        }
    }
}

int32_t motor_get_average_ticks(void)
{
    int32_t avg_ticks = 0;

    taskENTER_CRITICAL();
    avg_ticks = (g_left_ticks + g_right_ticks) / 2;
    taskEXIT_CRITICAL();

    return avg_ticks;
}

int32_t motor_get_left_ticks(void)
{
    int32_t ticks = 0;

    taskENTER_CRITICAL();
    ticks = g_left_ticks;
    taskEXIT_CRITICAL();

    return ticks;
}

int32_t motor_get_right_ticks(void)
{
    int32_t ticks = 0;

    taskENTER_CRITICAL();
    ticks = g_right_ticks;
    taskEXIT_CRITICAL();

    return ticks;
}

void motor_reset_ticks(void)
{
    taskENTER_CRITICAL();
    g_left_ticks  = 0;
    g_right_ticks = 0;
    taskEXIT_CRITICAL();
}

void vMotionControlTask(void *p_params)
{
    (void)p_params;

    (void)motor_setup(PWM_WRAP_VALUE, MAX_SAFE_SPEED_PWM, false);

    TickType_t last_wake_time = xTaskGetTickCount();
    uint16_t   packed_cmd     = 0U;
    uint8_t    active_mode    = MOT_CMD_FORWARD;
    uint16_t   base_speed     = 0U;
    uint8_t    stall_counter  = 0U;
    int32_t    turn_start_l   = 0;
    int32_t    prev_l_ticks   = 0;
    int32_t    prev_r_ticks   = 0;
    int32_t    integral_err   = 0;
    int32_t    prev_err       = 0;

    for (;;)
    {
        if ((NULL != g_motor_queue) &&
            (pdPASS == xQueueReceive(g_motor_queue, &packed_cmd, (TickType_t)0U)))
        {
            active_mode  = (uint8_t)((packed_cmd >> 14U) & 0x03U);
            base_speed   = (uint16_t)(packed_cmd & 0x3FFFU);
            turn_start_l = g_left_ticks;
        }

        int32_t curr_l  = g_left_ticks;
        int32_t curr_r  = g_right_ticks;
        int32_t delta_l = abs(curr_l - prev_l_ticks);
        int32_t delta_r = abs(curr_r - prev_r_ticks);
        prev_l_ticks    = curr_l;
        prev_r_ticks    = curr_r;

        if ((base_speed > 0U) && (MOT_CMD_FORWARD != active_mode))
        {
            int32_t turn_progress = abs(curr_l - turn_start_l);
            int32_t target_ticks  = (MOT_CMD_UTURN == active_mode) ? UTURN_180_TICKS : TURN_90_TICKS;

            if (turn_progress >= target_ticks)
            {
                active_mode = MOT_CMD_FORWARD;
            }
        }

        if (base_speed > (g_motor_cfg.max_speed / 2U))
        {
            if ((0 == delta_l) || (0 == delta_r))
            {
                stall_counter++;
                if (stall_counter >= STALL_CHECK_CYCLES)
                {
                    taskENTER_CRITICAL();
                    g_system_faults |= (uint8_t)(FAULT_MOTOR_STALL_BIT | FAULT_ENCODER_BIT);
                    taskEXIT_CRITICAL();
                    base_speed = 0U;
                }
            }
            else
            {
                stall_counter = 0U;
            }
        }

        if (0U == base_speed)
        {
            motor_set_raw(MOT_CMD_FORWARD, 0U, 0U);
            integral_err = 0;
            prev_err     = 0;
        }
        else
        {
            int32_t error = delta_l - delta_r;
            integral_err += error;
            int32_t derivative = error - prev_err;
            prev_err = error;

            int32_t correction = ((KP_SCALED * error) +
                                  (KI_SCALED * integral_err) +
                                  (KD_SCALED * derivative)) >> SCALE_SHIFT;

            int32_t pwm_l_calc = (int32_t)base_speed - correction;
            int32_t pwm_r_calc = (int32_t)base_speed + correction;

            if (pwm_l_calc < 0)
            {
                pwm_l_calc = 0;
            }

            if (pwm_r_calc < 0)
            {
                pwm_r_calc = 0;
            }

            motor_set_raw(active_mode, (uint16_t)pwm_l_calc, (uint16_t)pwm_r_calc);
        }

        (void)mqtt_enqueue_telemetry(2U, (uint32_t)packed_cmd);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PID_PERIOD_MS));
    }
}
