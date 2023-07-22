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

/**
 * @brief 主函数
 * 
 */
void app_main(void)
{
    // esp_err_t ret;
    vTaskPrioritySet(NULL, 16);
    
    pwr_ctrl_init();    // 2
    servo_init();       // 3
    sound_init();       // 6
    oled_main();        // 10   周期刷新数据
    argb_init();        // 12   周期刷新数据
    hid_host_init();    // 15

    vTaskPrioritySet(NULL, 1);

    char *buff = (char *)malloc(1024);
    while (1) {
        volatile const struct XboxData *xbox = get_xbox_pad_data();
        if (xbox->bnt3.share == 1) {
            /* 打印当前任务列表 */
            vTaskList(buff);
            printf("task\t\tstate\tprio\tstack\ttid\tcore\n");
            printf("%s\n", buff);
            /* 打印任务运行信息 */
            vTaskGetRunTimeStats(buff);
            printf("task_name\trun_cnt\t\tusage\n");
            printf("%s\n", buff);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    free(buff);

    vTaskDelete(NULL);
}

