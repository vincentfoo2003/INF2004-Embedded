#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "motor_pid_driver.h"

#define TEST_SPEED_PWM  (10000U)

static void vMotorTestSequenceTask(void *p_params)
{
    (void)p_params;
    uint16_t cmd = 0U;

    vTaskDelay(pdMS_TO_TICKS(2000U));
    printf("\n=== MOTOR PID DRIVER TEST STARTING ===\n");

    for (;;)
    {
        /* 1. Drive Forward */
        cmd = PACK_MOTOR_CMD(MOT_CMD_FORWARD, TEST_SPEED_PWM);
        (void)xQueueSend(g_motor_queue, &cmd, 0U);
        printf("[TEST] Mode: FORWARD | Speed: %u | Enc L: %ld | Enc R: %ld | Faults: 0x%02X\n",
               TEST_SPEED_PWM, motor_get_left_ticks(), motor_get_right_ticks(), g_system_faults);
        vTaskDelay(pdMS_TO_TICKS(3000U));

        /* 2. Stop */
        cmd = PACK_MOTOR_CMD(MOT_CMD_FORWARD, 0U);
        (void)xQueueSend(g_motor_queue, &cmd, 0U);
        printf("[TEST] Mode: STOP    | Enc L: %ld | Enc R: %ld\n",
               motor_get_left_ticks(), motor_get_right_ticks());
        vTaskDelay(pdMS_TO_TICKS(1000U));

        /* 3. Left Turn */
        cmd = PACK_MOTOR_CMD(MOT_CMD_LEFT, TEST_SPEED_PWM);
        (void)xQueueSend(g_motor_queue, &cmd, 0U);
        printf("[TEST] Mode: LEFT    | Speed: %u | Enc L: %ld | Enc R: %ld\n",
               TEST_SPEED_PWM, motor_get_left_ticks(), motor_get_right_ticks());
        vTaskDelay(pdMS_TO_TICKS(2000U));

        /* 4. Stop */
        cmd = PACK_MOTOR_CMD(MOT_CMD_FORWARD, 0U);
        (void)xQueueSend(g_motor_queue, &cmd, 0U);
        vTaskDelay(pdMS_TO_TICKS(2000U));
    }
}

int main(void)
{
    stdio_init_all();

    g_motor_queue = xQueueCreate(8U, sizeof(uint16_t));

    (void)xTaskCreate(vMotionControlTask, "MotionPID", 1024U, NULL, 2U, NULL);
    (void)xTaskCreate(vMotorTestSequenceTask, "MotorTest", 1024U, NULL, 1U, NULL);

    vTaskStartScheduler();

    for (;;)
    {
    }
    return 0;
}
