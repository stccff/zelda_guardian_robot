/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_system.h"
#include "esp_check.h"

#include "hid_host_gamepad.h"


/* I2S port and GPIOs */
#define I2S_NUM         (0)
#define I2S_MCK_IO      (-1)
#define I2S_BCK_IO      (GPIO_NUM_48)
#define I2S_WS_IO       (GPIO_NUM_45)
#define I2S_DO_IO       (GPIO_NUM_47)
#define I2S_DI_IO       (-1)
/* Example configurations */
// #define EXAMPLE_SAMPLE_RATE     (16000)
#define EXAMPLE_SAMPLE_RATE     (32000)


static const char *TAG = "sound";
static const char err_reason[][30] = {"input param is invalid",
                                      "operation timeout"
                                     };
static i2s_chan_handle_t g_tx_handle = NULL;

/* Import music file as buffer */
extern const uint8_t sight_search_start[] asm("_binary_sight_search_pcm_start");
extern const uint8_t sight_search_end[]   asm("_binary_sight_search_pcm_end");

/**
 * @brief i2s驱动初始化
 * 
 * @return esp_err_t 
 */
static esp_err_t i2s_driver_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &g_tx_handle, NULL));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_MCK_IO,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    // std_cfg.clk_cfg.mclk_multiple = EXAMPLE_MCLK_MULTIPLE;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(g_tx_handle, &std_cfg));
    // ESP_ERROR_CHECK(i2s_channel_enable(g_tx_handle));

    return ESP_OK;
}

static void i2s_play_sound(const void *src, size_t size, bool sw)
{
    esp_err_t ret = ESP_OK;
    size_t bytes_write = 0;

    static bool lastStatus = false;
    if (lastStatus != sw) { // 判断开关状态是否切换
        ESP_ERROR_CHECK((sw == true) ? i2s_channel_enable(g_tx_handle) : i2s_channel_disable(g_tx_handle));
    }
    lastStatus = sw;

    if (sw == false) {
        return; // 关闭播放直接返回
    }

    /* Write music to speaker */
    ret = i2s_channel_write(g_tx_handle, src, size, &bytes_write, portMAX_DELAY);
    if (ret != ESP_OK) {
        /* Since we set timeout to 'portMAX_DELAY' in 'i2s_channel_write'
        so you won't reach here unless you set other timeout value,
        if timeout detected, it means write operation failed. */
        ESP_LOGE(TAG, "[music] i2s write failed, %s", err_reason[ret == ESP_ERR_TIMEOUT]);
        abort();
    }
    if (bytes_write > 0) {
        // ESP_LOGI(TAG, "[music] i2s music played, %d bytes are written.", bytes_write);
    } else {
        ESP_LOGE(TAG, "[music] i2s music play falied.");
        abort();
    }
}


/**
 * @brief 声音播放任务
 * 
 * @param args 
 */
static void i2s_music_task(void *args)
{
    while (1) {
        const struct XboxData *xbox = get_xbox_pad_data();
        bool sw = (xbox->trigRT > (XBOX_TRIGGER_MAX / 2) ? true : false);
        i2s_play_sound(sight_search_start, sight_search_end - sight_search_start, sw);
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void sound_init(void)
{
    /* Initialize i2s peripheral */
    if (i2s_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "i2s driver init failed");
        abort();
    } else {
        ESP_LOGI(TAG, "i2s driver init success");
    }

    xTaskCreate(i2s_music_task, "i2s_music_task", 4096, NULL, 6, NULL);
}
