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



void app_main(void)
{
    // esp_err_t ret;
    oled_main();
    hid_host_init();
    servo_main();
}
