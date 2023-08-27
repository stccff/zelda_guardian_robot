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

#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "common_func.h"


const static char *TAG = "PWR_CTRL";

#define GPIO_PWR_CTRL   GPIO_NUM_9
#define GPIO_LASER_CTRL GPIO_NUM_37
#define GPIO_FUNC0      GPIO_NUM_46
#define TASK_PERIOD     (30)    // task周期（ms）
#define ESP_INTR_FLAG_DEFAULT 0
#define POWER_OFF_DELAY (60)    // s
#define ADC_SAMPLE_PERIOD (10)  // s
#define BAT_ADC1_CHN    ADC_CHANNEL_2

#define BAT_ADC 1


enum ClickState {
    NO_CLICK,
    WAIT_RASING,
    WAIT_CHECKED,
    SINGLE_CLICK,
    DOUBLE_CLICK,
    LONG_PRESS,
};

// 全局变量
static QueueHandle_t g_gpioEventQueue = NULL;

static adc_oneshot_unit_handle_t g_adc1_handle = NULL;
static adc_cali_handle_t g_adc1_cali_handle = NULL;
static bool g_do_calibration = false;

// 函数声明
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

/**
 * @brief adc处理程序
 * 
 * @param cnt 任务计数
 */
void adc_ctrl(uint32_t cnt)
{
    if ((cnt % ((ADC_SAMPLE_PERIOD * 1000) / TASK_PERIOD)) == 0) {
        int adc_raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(g_adc1_handle, BAT_ADC1_CHN, &adc_raw));
        // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, BAT_ADC1_CHN, adc_raw);
        if (g_do_calibration) {
            int voltage = 0;
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(g_adc1_cali_handle, adc_raw, &voltage));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, BAT_ADC1_CHN, voltage * 2);
        }
    }

    //Tear Down
    // ESP_ERROR_CHECK(adc_oneshot_del_unit(g_adc1_handle));
    // if (g_do_calibration) {
    //     example_adc_calibration_deinit(g_adc1_cali_handle);
    // }
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
}


/**
 * @brief gpio中断处理函数
 * 
 * @param arg 
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint64_t time = DRV_GetTime();
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
    if ((xbox->trigRT > (XBOX_TRIGGER_MAX / 20)) || (xbox->trigLT > (XBOX_TRIGGER_MAX / 2))) {
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


static void try_power_off(void)
{
    ESP_LOGI(TAG, "try to power off!");
    single_click_simulation();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    single_click_simulation();
    ESP_LOGE(TAG, "power off fail!!!");
}

/**
 * @brief 检查是否触发关机
 * 
 */
static void check_power_off(void)
{
    volatile const struct XboxData *xbox = get_xbox_pad_data();
    enum LightStatus status = sm_get_light_status();   // 获取灯光状态
    static enum LightStatus lastStatus = SM_LIGHT_NUN;
    static uint64_t pwrOffTime = 0;

    if (xbox->bnt2.btnSel == 1) {
        try_power_off();
    } 
    if (status == SM_NO_LIGHT) {
        if (lastStatus != status) {
            pwrOffTime = POWER_OFF_DELAY + DRV_GetTime() / 1000000; // 秒
        }
        if (DRV_GetTime() / 1000000 == pwrOffTime) {
            try_power_off();
        }   
    }

    lastStatus = status;
}

void static key_func0_ctrl(void)
{
    static uint64_t lastTime = 0;
    static enum ClickState state = 0;

    /* 按键中断处理 */
    if (state == WAIT_RASING) {
        if (DRV_GetTime() - lastTime > 2*1000*1000) { // 3s
            state = LONG_PRESS;
        }
    }
    if (state == WAIT_CHECKED) {
        if (DRV_GetTime() - lastTime > 500*1000) { // 500ms
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
        /* func0按键检测控制 */
        key_func0_ctrl();
        /* 激光控制 */
        laser_ctrl();
        /* 关机控制 */
        check_power_off();
        /* 模拟单击key，防止ip5306在低功耗自动关机 */
        if ((cnt % (10 * 1000 / TASK_PERIOD)) == 0) {    // 10s
            ESP_LOGI(TAG, "pwr heat beat");
            single_click_simulation();
        }
        /* adc检测 */
#if BAT_ADC
        adc_ctrl(cnt);
#endif

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

    /* 电压检测adc初始化 */
#if BAT_ADC
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &g_adc1_handle));
    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1_handle, BAT_ADC1_CHN, &config));
    //-------------ADC1 Calibration Init---------------//
    g_do_calibration = example_adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &g_adc1_cali_handle);
#endif

    xTaskCreate(pwr_ctrl_task, "pwr_ctrl_task", 4096, NULL, 2, NULL);
    // xTaskCreatePinnedToCore(pwr_ctrl_task, "pwr_ctrl_task", 2048, NULL, 1, NULL,  1);
}
