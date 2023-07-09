/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "hid_host_gamepad.h"


#define GPIO_PWR_CTRL   GPIO_NUM_9
#define GPIO_LASER_CTRL GPIO_NUM_37

#define TASK_PERIOD (50)    // ms

/**
 * @brief 激光打光控制
 * 
 */
static void laser_ctrl(void)
{
    const struct XboxData *xbox = get_xbox_pad_data();
    uint32_t level = (xbox->trigRT > (XBOX_TRIGGER_MAX / 2)) ? 1 : 0;
    gpio_set_level(GPIO_LASER_CTRL, level);
}

/**
 * @brief 模拟单击操作
 * 
 */
static void single_click_simulation(void)
{
    gpio_set_level(GPIO_PWR_CTRL, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_PWR_CTRL, 1);
}

/**
 * @brief 检查是否触发关机
 * 
 */
static void check_power_off(void)
{
    volatile const struct XboxData *xbox = get_xbox_pad_data();
    if (xbox->bnt3.share == 1) {
        printf("double click, power off!\n");
        single_click_simulation();
        vTaskDelay(600 / portTICK_PERIOD_MS);
        single_click_simulation();
    }
}

/**
 * @brief 电源控制主任务
 * 
 */
static void pwr_ctrl_task(void* arg)
{
    int cnt = 0;
    while (1) {
        cnt++;

        /* 激光控制 */
        laser_ctrl();
        /* 关机控制 */
        check_power_off();
        /* 模拟单击key，防止ip5306在低功耗自动关机 */
        if (cnt % (10 * 1000 / TASK_PERIOD) == 0) {    // 10s
            printf("single click\n");
            single_click_simulation();
        }

        vTaskDelay(TASK_PERIOD / portTICK_PERIOD_MS);
    }
}

/**
 * @brief 电源控制初始化
 * 
 */
void pwr_ctrl_init(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL << GPIO_PWR_CTRL) | (1ULL << GPIO_LASER_CTRL));
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));


    ESP_ERROR_CHECK(gpio_set_level(GPIO_PWR_CTRL, 1));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_LASER_CTRL, 0));

    xTaskCreate(pwr_ctrl_task, "pwr_ctrl_task", 4096, NULL, 3, NULL);
    // xTaskCreatePinnedToCore(pwr_ctrl_task, "pwr_ctrl_task", 2048, NULL, 1, NULL,  1);
}
