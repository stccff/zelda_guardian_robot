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

char taskList[512];

void app_main(void)
{
    // esp_err_t ret;
    hid_host_init();
    servo_main();
    oled_main();

    /* 打印当前 */
    vTaskList(taskList);
    printf("task\t\tstate\tprio\tstack\tnum\tcore\n");
    printf(taskList);
}
