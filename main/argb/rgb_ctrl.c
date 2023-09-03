/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include <stdlib.h>
#include "hid_host_gamepad.h"
#include "status_machine.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_HEAD_GPIO_NUM 36
#define RMT_LED_STRIP_BODY_GPIO_NUM 8
// #define RMT_LED_STRIP_TYPE_NUM      3   // blue, red, rgb       

#define LED_NUM     13
#define PERIOD_MS   40

static const char *TAG = "argb";
static uint8_t g_led_strip_pixels[LED_NUM][3];
static rmt_channel_handle_t g_led_chan1 = NULL;
static rmt_channel_handle_t g_led_chan2 = NULL;
static rmt_encoder_handle_t g_led_encoder1 = NULL;
static rmt_encoder_handle_t g_led_encoder2 = NULL;
// static uint8_t g_channel_sw = 0;

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
static void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

/**
 * @brief argb灯带设置固定颜色
 * 
 */
static void argb_color(uint16_t h, uint16_t s, uint16_t v)
{
    for (int j = 0; j < LED_NUM; j++) {
        // Build RGB pixels
        uint32_t red = 0;
        uint32_t green = 0;
        uint32_t blue = 0;
        led_strip_hsv2rgb(h, s, v, &red, &green, &blue);
        g_led_strip_pixels[j][0] = green;
        g_led_strip_pixels[j][1] = red;
        g_led_strip_pixels[j][2] = blue;
    }
}

/**
 * @brief argb灯带设置流水灯效
 * 
 */
static void argb_flowing(uint16_t v)
{
    static uint16_t start_rgb = 100;
    for (int j = 0; j < LED_NUM; j++) {
        // Build RGB pixels
        uint16_t hue = j * 360 / LED_NUM + start_rgb;
        uint32_t red = 0;
        uint32_t green = 0;
        uint32_t blue = 0;
        led_strip_hsv2rgb(hue, 100, v, &red, &green, &blue);
        g_led_strip_pixels[j][0] = green;
        g_led_strip_pixels[j][1] = red;
        g_led_strip_pixels[j][2] = blue;
    }
    start_rgb += PERIOD_MS / 10;
}


/**
 * @brief 电源控制主任务
 * 
 */
static void argb_ctrl_task(void* arg)
{
    enum LightStatus lastSm = false;

    // ESP_LOGI(TAG, "Enable RMT TX channel");
    // ESP_ERROR_CHECK(rmt_enable(g_led_chan1));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };

    while (1) {
        /* 处理手柄数据 */
        const struct XboxData *xbox = get_xbox_pad_data();
        enum LightStatus status = sm_get_light_status();
        if (xbox->DPad == 1) {   // dpad上
            if (status < SM_LIGHT_NUN - 1) {
                status++;
                sm_set_light_status(status);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        } else if (xbox->DPad == 5) {   // dpad下
            if (status > 0) {
                status--;
                sm_set_light_status(status);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        
        /* 开关通道 */
        if (lastSm != status) {
            if ((lastSm <= SM_LIGHT_SCREEN) && (status > SM_LIGHT_SCREEN)) {
                ESP_LOGI(TAG, "rmt_enable");
                ESP_ERROR_CHECK(rmt_enable(g_led_chan1));
                ESP_ERROR_CHECK(rmt_enable(g_led_chan2));
            } else if ((lastSm > SM_LIGHT_SCREEN) && (status <= SM_LIGHT_SCREEN)) {
                ESP_LOGI(TAG, "rmt_disable");
                memset(g_led_strip_pixels, 0, sizeof(g_led_strip_pixels));
                // Flush RGB values to LEDs
                ESP_ERROR_CHECK(rmt_transmit(g_led_chan1, g_led_encoder1, g_led_strip_pixels, sizeof(g_led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_transmit(g_led_chan2, g_led_encoder2, g_led_strip_pixels, sizeof(g_led_strip_pixels), &tx_config));
                vTaskDelay(pdMS_TO_TICKS(10)); // 等待数据传输完成
                ESP_ERROR_CHECK(rmt_disable(g_led_chan1));
                ESP_ERROR_CHECK(rmt_disable(g_led_chan2));
            }
            lastSm = status;
        }
        
        /* 填充rgb数据 */
        if (status > SM_LIGHT_SCREEN) {
            /* 构造rgb数据 */
            switch (status) {
                case SM_LIGHT_SCREEN_BLUE:
                    argb_color(240, 100, 60);   // blue
                    break;
                case SM_LIGHT_SCREEN_RED:
                    argb_color(320, 100, 40);   // red
                    break;
                case SM_LIGHT_SCREEN_ARGB:
                    argb_flowing(40);
                    break;
                default:
                    break;
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(rmt_transmit(g_led_chan1, g_led_encoder1, g_led_strip_pixels, sizeof(g_led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_transmit(g_led_chan2, g_led_encoder2, g_led_strip_pixels, sizeof(g_led_strip_pixels), &tx_config));
        }
        

        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
    };
}


void argb_init(void)
{
    ESP_LOGI(TAG, "Create RMT TX channel");

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_HEAD_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &g_led_chan1));

    tx_chan_config.gpio_num = RMT_LED_STRIP_BODY_GPIO_NUM;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &g_led_chan2));

    ESP_LOGI(TAG, "Install led strip encoder");
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &g_led_encoder1));
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &g_led_encoder2));

    /* 清除rgb灯珠中的数据（未完全下电情况下，数据并未被清除） */
    memset(g_led_strip_pixels, 0, sizeof(g_led_strip_pixels));
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
    ESP_ERROR_CHECK(rmt_enable(g_led_chan1));
    ESP_ERROR_CHECK(rmt_enable(g_led_chan2));
    ESP_ERROR_CHECK(rmt_transmit(g_led_chan1, g_led_encoder1, g_led_strip_pixels, sizeof(g_led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_transmit(g_led_chan2, g_led_encoder2, g_led_strip_pixels, sizeof(g_led_strip_pixels), &tx_config));
    vTaskDelay(pdMS_TO_TICKS(10)); // 等待数据传输完成
    ESP_ERROR_CHECK(rmt_disable(g_led_chan1));
    ESP_ERROR_CHECK(rmt_disable(g_led_chan2));

    xTaskCreate(argb_ctrl_task, "argb_ctrl_task", 4096, NULL, 12, NULL);
}



