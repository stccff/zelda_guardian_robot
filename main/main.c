/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "hid_host_gamepad.h"
#include "oled_main.h"
#include "servo_control.h"
#include "i2s_std_sound.h"
#include "pwr_ctrl.h"
#include "rgb_ctrl.h"


char g_taskList[1024];


/**
 * @brief 主函数
 * 
 */
void app_main(void)
{
    // esp_err_t ret;
    vTaskPrioritySet(NULL, 16);

    argb_init();
    pwr_ctrl_init();
    oled_main();
    sound_init();
    servo_init();
    hid_host_init();

    /* 打印当前 */
    vTaskList(g_taskList);
    printf("task\t\tstate\tprio\tstack\tnum\tcore\n");
    printf(g_taskList);

    // while (1) {
    //     vTaskDelay(10 / portTICK_PERIOD_MS);
    // }
    vTaskDelete(NULL);
}
