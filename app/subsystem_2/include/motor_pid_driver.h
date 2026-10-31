#ifndef MOTOR_PID_DRIVER_H
#define MOTOR_PID_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "queue.h"

/* Hardware Pin Definitions (Cytron Robo Pico Final Assembled Layout) */
#define PIN_ENC_L_A             (2U)
#define PIN_ENC_L_B             (3U)
#define PIN_ENC_R_A             (4U)
#define PIN_ENC_R_B             (5U)

#define PIN_MOT_L_M1A           (8U)
#define PIN_MOT_L_M1B           (9U)
#define PIN_MOT_R_M2A           (10U)
#define PIN_MOT_R_M2B           (11U)

#define PIN_IMU_INT             (22U)
#define PIN_IR_R_BARCODE        (27U)

/* Fault Bitmask Definitions */
#define FAULT_MOTOR_STALL_BIT   (1U << 0U)
#define FAULT_ENCODER_BIT       (1U << 1U)

/* Motor Command Modes (2-bit field in 16-bit packed word) */
#define MOT_CMD_FORWARD         (0U)
#define MOT_CMD_LEFT            (1U)
#define MOT_CMD_RIGHT           (2U)
#define MOT_CMD_UTURN           (3U)

#define PACK_MOTOR_CMD(mode, speed) \
    ((uint16_t)((((uint16_t)(mode) & 0x03U) << 14U) | ((uint16_t)(speed) & 0x3FFFU)))

/* PID & PWM Constants */
#define PID_PERIOD_MS           (20U)
#define PWM_WRAP_VALUE          (20000U)
#define MAX_SAFE_SPEED_PWM      (16383U)
#define STALL_CHECK_CYCLES      (25U)

#define TURN_90_TICKS           (450)
#define UTURN_180_TICKS         (900)

struct motor_config_s {
    uint16_t pwm_wrap;
    uint16_t max_speed;
    bool log;
} typedef motor_config_t;

extern volatile uint8_t g_system_faults;
extern QueueHandle_t    g_motor_queue;

motor_config_t motor_setup(uint16_t pwm_wrap, uint16_t max_speed, bool log);
void motor_set_raw(uint8_t cmd_mode, uint16_t pwm_left, uint16_t pwm_right);
int32_t motor_get_average_ticks(void);
int32_t motor_get_left_ticks(void);
int32_t motor_get_right_ticks(void);
void motor_reset_ticks(void);
void master_gpio_callback(uint gpio, uint32_t events);
void vMotionControlTask(void *p_params);

#endif
