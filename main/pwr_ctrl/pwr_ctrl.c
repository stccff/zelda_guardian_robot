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
#include "driver/gpio_filter.h"
#include "hid_host_gamepad.h"
#include "time.h"
#include "status_machine.h"


#define GPIO_PWR_CTRL   GPIO_NUM_9
#define GPIO_LASER_CTRL GPIO_NUM_37
#define GPIO_FUNC0      GPIO_NUM_46
#define TASK_PERIOD     (30)    // task周期（ms）
#define ESP_INTR_FLAG_DEFAULT 0
#define POWER_OFF_DELAY (60)    // s


enum ClickState {
    NO_CLICK,
    WAIT_RASING,
    WAIT_CHECKED,
    SINGLE_CLICK,
    DOUBLE_CLICK,
    LONG_PRESS,
};


static QueueHandle_t g_gpioEventQueue = NULL;


static uint64_t drv_gettime(void)
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return (uint64_t)tp.tv_sec * 1000000 + (uint64_t)tp.tv_nsec / 1000;
}


/**
 * @brief gpio中断处理函数
 * 
 * @param arg 
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint64_t time = drv_gettime();
    xQueueSendFromISR(g_gpioEventQueue, &time, NULL);
}

/**
 * @brief 激光打光控制
 * 
 */
static void laser_ctrl(void)
{
    const struct XboxData *xbox = get_xbox_pad_data();
    uint32_t level;
    if ((xbox->trigRT > (XBOX_TRIGGER_MAX / 2)) || (xbox->trigLT > (XBOX_TRIGGER_MAX / 2))) {
        level = 1;
    } else {
        level = 0;
    }
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
    enum LightStatus lightStatus = sm_get_light_status();   // 获取灯光状态

    if (xbox->bnt2.btnSel == 1) {
        goto pwr_off;
    } else if (lightStatus == SM_NO_LIGHT) {
        struct timespec tp;
        clock_gettime(CLOCK_MONOTONIC, &tp);
        uint64_t currTime = tp.tv_sec * 1000000 + (uint64_t)tp.tv_nsec / 1000;
        if (currTime / 1000000 == POWER_OFF_DELAY) {    // todo 从进入nolight模式开始计时
            goto pwr_off;
        }
        return;
    } else {
        return;
    }

pwr_off:
    printf("try power off!\n");
    single_click_simulation();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    single_click_simulation();
    printf("power off fail!!!\n");
}

/**
 * @brief 电源控制主任务
 * 
 */
static void pwr_ctrl_task(void* arg)
{
    int cnt = 0;
    int posNum = 0;
    int negNum = 0;
    uint64_t lastTime = 0;

    enum ClickState state = 0;

    while (1) {
        cnt++;
        /* 按键中断处理 */
        if (state == WAIT_RASING) {
            if (drv_gettime() - lastTime > 2*1000*1000) { // 3s
                state = LONG_PRESS;
            }
        }
        if (state == WAIT_CHECKED) {
            if (drv_gettime() - lastTime > 500*1000) { // 500ms
                state = SINGLE_CLICK;
            }
        }

        uint64_t time;
        if (xQueueReceive(g_gpioEventQueue, &time, 0)) {
            vTaskDelay(10/ portTICK_PERIOD_MS);    // 延时去抖（抖动范围实测几十~几百us）
            uint64_t tmp;
            while (xQueueReceive(g_gpioEventQueue, &tmp, 0));    // 清空队列
            if (gpio_get_level(GPIO_FUNC0) == 0) {
                if (state == WAIT_CHECKED) {
                    if (time - lastTime < 400*1000) {
                        state = DOUBLE_CLICK;
                    }
                } else {
                    state = WAIT_RASING;
                    lastTime = time;
                }
                // printf("[%llu] falling click, num = %d\n", time, negNum++);
            } else {
                if (state == WAIT_RASING) {
                    state = WAIT_CHECKED;
                } else {
                    state = NO_CLICK;
                }
                // printf("[%llu] rasing click, num = %d\n", time, posNum++);
            }
        }

        
        switch (state) {
            case SINGLE_CLICK:
                state = NO_CLICK;
                /* 单击事件处理 */
                enum LightStatus lightStatus = sm_get_light_status();   // 获取灯光状态
                lightStatus++;
                if (lightStatus >= SM_LIGHT_NUN) {
                    lightStatus = 0;
                }
                sm_set_light_status(lightStatus);
                printf("[%llu] single click\n", time);
                break;
            case DOUBLE_CLICK:
                state = NO_CLICK;
                printf("[%llu] double click\n", time);
                break;
            case LONG_PRESS:
                state = NO_CLICK;
                sm_set_bluetooth_status(sm_get_bluetooth_status() == SM_BL_SCAN_OFF ? SM_BL_SCAN_ON : SM_BL_SCAN_OFF);
                printf("[%llu] long press\n", time);
                break;
            default:
                break;
        }


        // while (xQueueReceive(g_gpioEventQueue, &time, 0)) {
        //     printf("[%llu] num = %d\n", time, num++);
        // }

        /* 激光控制 */
        laser_ctrl();
        /* 关机控制 */
        check_power_off();
        /* 模拟单击key，防止ip5306在低功耗自动关机 */
        if ((cnt % (10 * 1000 / TASK_PERIOD)) == 0) {    // 10s
            printf("pwr heat beat\n");
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
    /* 配置电源控制与激光控制IO */
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

    /* 配置FUNC0功能按键IO */
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins
    io_conf.pin_bit_mask = 1ULL << GPIO_FUNC0;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    g_gpioEventQueue = xQueueCreate(16, sizeof(uint64_t));

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);    // 与中断优先级相关？
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_FUNC0, gpio_isr_handler, NULL);

    /* 设置gpio按键防抖滤波, 当前sdk存在bug */
    // gpio_glitch_filter_handle_t filter_handle = NULL;

    // gpio_flex_glitch_filter_config_t filterCfg = {
    //     .clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT,
    //     .gpio_num = GPIO_FUNC0,
    //     .window_thres_ns = 50*1000,
    //     .window_width_ns = 100*1000,
    // };
    // ESP_ERROR_CHECK(gpio_new_flex_glitch_filter(&filterCfg, &filter_handle));

    // gpio_pin_glitch_filter_config_t filterCfg = {
    //     .clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT,
    //     .gpio_num = GPIO_FUNC0,
    // };
    // ESP_ERROR_CHECK(gpio_new_pin_glitch_filter(&filterCfg, &filter_handle));

    // ESP_ERROR_CHECK(gpio_glitch_filter_enable(filter_handle));


    xTaskCreate(pwr_ctrl_task, "pwr_ctrl_task", 4096, NULL, 2, NULL);
    // xTaskCreatePinnedToCore(pwr_ctrl_task, "pwr_ctrl_task", 2048, NULL, 1, NULL,  1);
}
