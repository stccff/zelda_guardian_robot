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
#include "status_machine.h"

static portMUX_TYPE g_spinLock = portMUX_INITIALIZER_UNLOCKED;

/*
 vTaskGetRunTimeStats() 使用
 注意:
 使用 vTaskGetRunTimeStats() 前需使能:
 make menuconfig -> Component config -> FreeRTOS -> configUSE_TRACE_FACILITY
 make menuconfig -> Component config -> FreeRTOS -> Enable FreeRTOS trace facility -> configUSE_STATS_FORMATTING_FUNCTIONS
 make menuconfig -> Component config -> FreeRTOS -> Enable display of xCorelD in vlaskList
 make menuconfig -> Component config -> FreeRTOS -> configGENERATE_RUN_TIME_STATS
 通过上面配置，等同于使能 FreeRTOSConfig.h 中如下三个宏:
 configGENERATE_RUN_TIME_STATS，configUSE_STATS_FORMATTING_FUNCTIONS 和 configSUPPORT_DYNAMIC_ALLOCATION
 */
void print_task_info(char *buff)
{
    /* 打印当前任务列表 */
    vTaskList(buff);
    printf("task\t\tstate\tprio\tstack\ttid\tcore\n");
    printf("%s\n", buff);
    /* 打印任务运行信息 */
    vTaskGetRunTimeStats(buff);
    printf("task_name\trun_cnt\t\tusage\n");
    printf("%s\n", buff);
}

static bool is_demo_mod(void)
{
    volatile const struct XboxData *xbox = get_xbox_pad_data();
    if (sm_get_active_mod() != SM_DEMO || xbox->bnt2.btnStart == 1) {
        vTaskDelay(200 / portTICK_PERIOD_MS);
        sm_set_active_mod(SM_NORMAL);
        struct XboxData virXbox = {
            .joyLx = 0xffff / 2,
            .joyRx = 0xffff / 2,
            .joyRy = 0xffff / 2,
            .trigRT = 0,
        };
        set_xbox_pad_data_virtual(&virXbox);
        oled_set_display(OLED_OFF);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        return false;
    }
    return true;
}

static bool demo_delay_and_check(uint32_t ms)
{
    bool is_demo;
    for (uint32_t i = 0; (i < ms / 20) && (is_demo = is_demo_mod() == true); i++) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    return is_demo;
}

/**
 * @brief demo sequence
 * 
 */
static void do_demo_sequence(void)
{
    /* sleep */
    sm_set_light_status(SM_NO_LIGHT);
    if (demo_delay_and_check(4000) == false) {
        return;
    }
    
    /* eye on */
    oled_set_display(OLED_CIRCLE);
    sm_set_light_status(SM_LIGHT_SCREEN);
    sound_play_eyeon(true);
    if (demo_delay_and_check(3000) == false) {
        return;
    }

    /* blue */
    sm_set_light_status(SM_LIGHT_SCREEN_BLUE);
    if (demo_delay_and_check(3000) == false) {
        return;
    }

    /* randon move */
    struct XboxData virXbox = {
        .joyLx = 0xffff / 2,
        .joyRx = 0xffff / 2,
        .joyRy = 0xffff / 2,
        .trigRT = 0,
    };
    virXbox.joyLx = XBOX_JOYSTICK_MAX / 2 - XBOX_JOYSTICK_MAX / 20 * 9;
    virXbox.joyRx = XBOX_JOYSTICK_MAX / 2 + XBOX_JOYSTICK_MAX / 20 * 9;
    set_xbox_pad_data_virtual(&virXbox);
    if (demo_delay_and_check(500) == false) {
        return;
    }

    virXbox.joyLx = XBOX_JOYSTICK_MAX / 2 + XBOX_JOYSTICK_MAX / 20 * 9;
    virXbox.joyRx = XBOX_JOYSTICK_MAX / 2 - XBOX_JOYSTICK_MAX / 20 * 9;
    set_xbox_pad_data_virtual(&virXbox);
    if (demo_delay_and_check(1000) == false) {
        return;
    }

    virXbox.joyLx = XBOX_JOYSTICK_MAX / 2 - XBOX_JOYSTICK_MAX / 20 * 9;
    virXbox.joyRx = XBOX_JOYSTICK_MAX / 2 + XBOX_JOYSTICK_MAX / 20 * 9;
    set_xbox_pad_data_virtual(&virXbox);
    if (demo_delay_and_check(1000) == false) {
        return;
    }

    virXbox.joyLx = XBOX_JOYSTICK_MAX / 2 + XBOX_JOYSTICK_MAX / 20 * 9;
    virXbox.joyRx = XBOX_JOYSTICK_MAX / 2 - XBOX_JOYSTICK_MAX / 20 * 9;
    set_xbox_pad_data_virtual(&virXbox);
    if (demo_delay_and_check(500) == false) {
        return;
    }
    
    // resume
    virXbox.joyLx = XBOX_JOYSTICK_MAX / 2;
    virXbox.joyRx = XBOX_JOYSTICK_MAX / 2;
    set_xbox_pad_data_virtual(&virXbox);

    /* attach */
    sm_set_light_status(SM_LIGHT_SCREEN_RED);
    if (demo_delay_and_check(1000) == false) {
        return;
    }
    virXbox.trigRT = XBOX_TRIGGER_MAX;
    set_xbox_pad_data_virtual(&virXbox);

    virXbox.joyRy = XBOX_JOYSTICK_MAX / 2 - XBOX_JOYSTICK_MAX / 20 * 6;
    set_xbox_pad_data_virtual(&virXbox);
    if (demo_delay_and_check(200) == false) {
        return;
    }

    virXbox.joyRy = XBOX_JOYSTICK_MAX / 2 + XBOX_JOYSTICK_MAX / 20 * 6;
    set_xbox_pad_data_virtual(&virXbox);
    if (demo_delay_and_check(400) == false) {
        return;
    }

    virXbox.joyRy = XBOX_JOYSTICK_MAX / 2 - XBOX_JOYSTICK_MAX / 20 * 6;
    set_xbox_pad_data_virtual(&virXbox);
    if (demo_delay_and_check(400) == false) {
        return;
    }

    virXbox.joyRy = XBOX_JOYSTICK_MAX / 2 + XBOX_JOYSTICK_MAX / 20 * 6;
    set_xbox_pad_data_virtual(&virXbox);
    if (demo_delay_and_check(400) == false) {
        return;
    }

    virXbox.joyRy = XBOX_JOYSTICK_MAX / 2 - XBOX_JOYSTICK_MAX / 20 * 6;
    set_xbox_pad_data_virtual(&virXbox);
    if (demo_delay_and_check(200) == false) {
        return;
    }

    // resume
    virXbox.joyRy = XBOX_JOYSTICK_MAX / 2;
    set_xbox_pad_data_virtual(&virXbox);

    if (demo_delay_and_check(4000) == false) {
        return;
    }
    
    virXbox.trigRT = 0;
    set_xbox_pad_data_virtual(&virXbox);
    if (demo_delay_and_check(5000) == false) {
        return;
    }

    sm_set_light_status(SM_LIGHT_SCREEN_BLUE);
    if (demo_delay_and_check(10000) == false) {
        return;
    }
}

/**
 * @brief 主函数
 * 
 */
void app_main(void)
{
    /* init process*/
    taskENTER_CRITICAL(&g_spinLock);
    status_machine_init();
    taskEXIT_CRITICAL(&g_spinLock);

    pwr_ctrl_init();    // 2
    servo_init();       // 3
    sound_init();       // 7/6
    oled_main();        // 10   周期刷新数据
    argb_init();        // 12   周期刷新数据
    hid_host_init();    // 15

    char *buff = (char *)malloc(1024);
    print_task_info(buff);

    while (1) {
        /* print task info */
        volatile const struct XboxData *xbox = get_xbox_pad_data();
        if (xbox->bnt3.share == 1) {
            print_task_info(buff);
        }

        /* demo mod detecte */
        if (xbox->bnt2.btnStart == 1) {
            vTaskDelay(300 / portTICK_PERIOD_MS);
            sm_set_active_mod(SM_DEMO);
        }

        /* demo auto play */
        while (sm_get_active_mod() == SM_DEMO) {
            do_demo_sequence();
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    free(buff);

    vTaskDelete(NULL);
}

