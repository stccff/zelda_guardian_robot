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
#include "esp_spiffs.h"
#include "mbedtls/md5.h"
#include "driver/gpio.h"

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#include "minimp3.h"
#include "minimp3_ex.h"


#define INPUT_BUFFER_SIZE (1024 * 16)

/* I2S port and GPIOs */
#define I2S_NUM         (0)
#define I2S_MCK_IO      (-1)
#define I2S_BCK_IO      (GPIO_NUM_48)
#define I2S_WS_IO       (GPIO_NUM_45)
#define I2S_DO_IO       (GPIO_NUM_47)
#define I2S_DI_IO       (-1)

#define AMPLIFIER_GAIN_IO   (GPIO_NUM_21)

/* Example configurations */
// #define EXAMPLE_SAMPLE_RATE     (16000)
#define EXAMPLE_SAMPLE_RATE     (48000)


/* 结构定义 */
struct Mp3Info {
    uint8_t *input_buf;
    int16_t *pcm_buf;
    uint32_t buffered;
    uint32_t to_read;
    FILE *fp;
    mp3dec_t mp3d;
    mp3dec_frame_info_t info;
};

static const char *TAG = "sound";
static const char err_reason[][30] = {"input param is invalid",
                                      "operation timeout"
                                     };
static i2s_chan_handle_t g_tx_handle = NULL;

/* Import music file as buffer */
// extern const uint8_t sight_search_start[] asm("_binary_sight_search_pcm_start");
// extern const uint8_t sight_search_end[]   asm("_binary_sight_search_pcm_end");


/**
 * @brief 设置硬件音量（MAX98357）
 * 
 * @param vol 
 */
static void set_hw_volume(uint32_t vol)
{
    gpio_mode_t gpioMod;
    uint32_t level;

    switch (vol) {
        case 0:
            gpioMod = GPIO_MODE_OUTPUT;
            level = 1;  // 上拉，6dB增益
            break;
        case 1:
            gpioMod = GPIO_MODE_OUTPUT_OD;   // 浮空，9dB增益
            level = 1;
            break;
        case 2:
            gpioMod = GPIO_MODE_OUTPUT;
            level = 0;  // 下拉，12dB增益
            break;
        default:
            ESP_LOGE(TAG, "invalid volume value!!!");
            return;
    }
    
    gpio_config_t io_conf = {}; //zero-initialize the config structure.
    io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
    io_conf.mode = gpioMod; //set as output mode
    io_conf.pin_bit_mask = (1ULL << AMPLIFIER_GAIN_IO); //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = 0;   //disable pull-down mode
    io_conf.pull_up_en = 0; //disable pull-up mode
    ESP_ERROR_CHECK(gpio_config(&io_conf)); //configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_set_level(AMPLIFIER_GAIN_IO, level));  
}

static void compute_alice_txt_md5(void)
{
    ESP_LOGI(TAG, "Computing alice.txt MD5 hash");

    // The file alice.txt lives under a subdirectory, though SPIFFS itself is flat
    // FILE* f = fopen("/spiffs/sub/alice.txt", "r");
    FILE* f = fopen("/spiffs/bad_apple.mp3", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open alice.txt");
        return;
    }

    // Read file and compute the digest chunk by chunk
    #define MD5_MAX_LEN 16

    char buf[64];
    mbedtls_md5_context ctx;
    unsigned char digest[MD5_MAX_LEN];

    mbedtls_md5_init(&ctx);
    mbedtls_md5_starts(&ctx);

    size_t read;

    do {
        read = fread((void*) buf, 1, sizeof(buf), f);
        mbedtls_md5_update(&ctx, (unsigned const char*) buf, read);
    } while(read == sizeof(buf));

    mbedtls_md5_finish(&ctx, digest);

    // Create a string of the digest
    char digest_str[MD5_MAX_LEN * 2];

    for (int i = 0; i < MD5_MAX_LEN; i++) {
        sprintf(&digest_str[i * 2], "%02x", (unsigned int)digest[i]);
    }

    // For reference, MD5 should be deeb71f585cbb3ae5f7976d5127faf2a
    ESP_LOGI(TAG, "Computed MD5 hash of alice.txt: %s", digest_str);

    fclose(f);
}

/**
 * @brief 从input_buff中，读取并解码MP3成pcm，存入pcm_buff中
 * 
 * @param fp 
 * @param mp3Info 
 * @return int 
 */
static int decode_mp3(struct Mp3Info *mp3Info)
{

    // mp3dec_frame_info_t info = {};
    int decoded = 0;

    // read in the data that is needed to top up the buffer
    size_t n = fread(mp3Info->input_buf + mp3Info->buffered, 1, mp3Info->to_read, mp3Info->fp);
    // ESP_LOGI(TAG, "Read %d bytes", n);
    // for (int i = 0; i < MINIMP3_MAX_SAMPLES_PER_FRAME * 2; i++) {
    //     if (i % 16 == 0) {
    //         printf("%08x\t", i);
    //     }
    //     printf("%02x\t", mp3Info->input_buf[i]);
    //     if (i % 16 == 15) {
    //         printf("\n");
    //     }
    // }
    if (n == 0) {
        fseek(mp3Info->fp, 0, SEEK_SET); // 重头继续读取 todo 可能有问题？？
    }
    mp3Info->buffered += n;
    // decode the next frame
    int samples = mp3dec_decode_frame(&mp3Info->mp3d, mp3Info->input_buf, mp3Info->buffered, mp3Info->pcm_buf, &mp3Info->info);
    // ESP_LOGI(TAG, "frame_bytes = %d", mp3Info->info.frame_bytes);
    // ESP_LOGI(TAG, "frame_offset = %d", mp3Info->info.frame_offset);
    // ESP_LOGI(TAG, "channels = %d", mp3Info->info.channels);
    // ESP_LOGI(TAG, "hz = %d", mp3Info->info.hz);
    // ESP_LOGI(TAG, "layer = %d", mp3Info->info.layer);
    // ESP_LOGI(TAG, "bitrate_kbps = %d", mp3Info->info.bitrate_kbps);
    // ESP_LOGI(TAG, "samples = %d", samples);
    // we've processed this may bytes from teh buffered data
    mp3Info->buffered -= mp3Info->info.frame_bytes;
    // shift the remaining data to the front of the buffer
    memmove(mp3Info->input_buf, mp3Info->input_buf + mp3Info->info.frame_bytes, mp3Info->buffered);
    // we need to top up the buffer from the file
    mp3Info->to_read = mp3Info->info.frame_bytes;
    if (samples > 0) {
        // keep track of how many samples we've decoded
        decoded += samples;
    }
    // ESP_LOGI(TAG, "decoded %d samples", decoded);
    // ESP_LOGI(TAG, "\n");
    // vTaskDelay(pdMS_TO_TICKS(10000));

    return samples;
}

/**
 * @brief spiffs初始化
 * 
 * @return esp_err_t 
 */
esp_err_t spiffs_init(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = false
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    return ret;
}



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

static void i2s_play_sound(struct Mp3Info *mp3Info, bool sw)
{
    esp_err_t ret = ESP_OK;
    size_t bytes_write = 0;

    static bool lastStatus = false;
    if (lastStatus != sw) { // 判断开关状态是否切换
        if (sw == true) {
            ESP_ERROR_CHECK(i2s_channel_enable(g_tx_handle));   // 使能通道

            mp3Info->fp = fopen("/spiffs/bad_apple.mp3", "r");     // 打开文件
            ESP_ERROR_CHECK(!mp3Info->fp);

            mp3Info->buffered = 0;
            mp3Info->to_read = INPUT_BUFFER_SIZE;   // 初始化变量
            memset(&mp3Info->mp3d, 0, sizeof(mp3dec_t));
            memset(&mp3Info->info, 0, sizeof(mp3dec_frame_info_t));
            mp3dec_init(&mp3Info->mp3d); // mp3 decoder state
        } else {
            ESP_ERROR_CHECK(i2s_channel_disable(g_tx_handle));

            fclose(mp3Info->fp);
        }
    }
    lastStatus = sw;

    if (sw == false) {
        return; // 关闭播放直接返回
    }

    int samples;
    do {
        samples = decode_mp3(mp3Info);    // 解码
    } while (!samples);
    
    
    /* Write music to speaker */
    size_t size = samples * mp3Info->info.channels * sizeof(short);
    ret = i2s_channel_write(g_tx_handle, mp3Info->pcm_buf, size, &bytes_write, portMAX_DELAY);
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

static void i2s_play_sound_ex(bool sw)
{   
    esp_err_t ret = ESP_OK;
    static mp3dec_ex_t dec;
    // static mp3dec_t mp3d;
    // static mp3dec_file_info_t info;
    static mp3d_sample_t *buffer;
    static size_t readed;

    static bool lastStatus = false;
    if (lastStatus != sw) { // 判断开关状态是否切换
        if (sw == true) {
            ESP_ERROR_CHECK(i2s_channel_enable(g_tx_handle));   // 使能通道
            
            // ret = mp3dec_load(&mp3d, "/spiffs/guardian_beamsightlocking.mp3", &info, NULL, NULL);
            // if (ret) {
            //     ESP_LOGE(TAG, "mp3dec_load fail! ret = %d\n", ret);
            // }
            /* mp3dec_file_info_t contains decoded samples and info, use free(info.buffer) to deallocate samples */
            ret = mp3dec_ex_open(&dec, "/spiffs/guardian_beamsightlocking.mp3", MP3D_SEEK_TO_SAMPLE);
            if (ret) {
                ESP_LOGE(TAG, "mp3dec_ex_open fail! ret = %d\n", ret);
                return;
            }
            /* dec.samples, dec.info.hz, dec.info.layer, dec.info.channels should be filled */
            ret = mp3dec_ex_seek(&dec, 5936);
            if (ret) {
                ESP_LOGE(TAG, "mp3dec_ex_seek error! ret = %d\n", ret);
                mp3dec_ex_close(&dec);
                return;
            }
            size_t size = dec.samples * sizeof(mp3d_sample_t);
            buffer = malloc(size);
            readed = mp3dec_ex_read(&dec, buffer, (17498 - 5936));
printf("readed = %d, samples = %llu", readed, dec.samples);
            /* normal eof or error condition */
            if (readed != dec.samples) {
                if (dec.last_error) {
                    ESP_LOGE(TAG, "last error!\n");
                }
            }

        } else {
            ESP_ERROR_CHECK(i2s_channel_disable(g_tx_handle));
            mp3dec_ex_close(&dec);
            // free(info.buffer);
            free(buffer);
        }
    }
    lastStatus = sw;

    if (sw == false) {
        return; // 关闭播放直接返回
    }

    /* Write music to speaker */
    size_t bytes_write = 0;
    ret = i2s_channel_write(g_tx_handle, buffer, readed * sizeof(mp3d_sample_t), &bytes_write, portMAX_DELAY);
    if (ret != ESP_OK) {
        /* Since we set timeout to 'portMAX_DELAY' in 'i2s_channel_write'
        so you won't reach here unless you set other timeout value,
        if timeout detected, it means write operation failed. */
        ESP_LOGE(TAG, "[music] i2s write failed, %s", err_reason[ret == ESP_ERR_TIMEOUT]);
    }
    if (bytes_write > 0) {
        // ESP_LOGI(TAG, "[music] i2s music played, %d bytes are written.", bytes_write);
    } else {
        ESP_LOGE(TAG, "[music] i2s music play falied.");
    }
}


/**
 * @brief 声音播放任务
 * 
 * @param args 
 */
static void i2s_music_task(void *args)
{

    // init buffer
    // struct Mp3Info mp3Info = {0};
    // mp3Info.pcm_buf = (short *)malloc(sizeof(short) * MINIMP3_MAX_SAMPLES_PER_FRAME);
    // ESP_ERROR_CHECK(!mp3Info.pcm_buf);
    // mp3Info.input_buf = (uint8_t *)malloc(INPUT_BUFFER_SIZE);
    // ESP_ERROR_CHECK(!mp3Info.input_buf);

    // compute_alice_txt_md5();
    uint32_t vol_cnt = 0;
    uint32_t vol = 0;
    while (1) {
        const struct XboxData *xbox = get_xbox_pad_data();
        // bool sw = (xbox->trigRT > (XBOX_TRIGGER_MAX / 2) ? true : false);
        bool sw = true;
        uint32_t delay = sw ? 1 : 100;

        /* 设置音量 */
        if ((vol_cnt > 2) || (delay == 100)) {
            vol_cnt = 0;
            if (xbox->DPad == 3) {
                if (vol < 2) {
                    vol++;
                    set_hw_volume(vol);
                    ESP_LOGI(TAG, "hw volume = %d", vol);
                }
            }
            if (xbox->DPad == 7) {
                if (vol > 0) {
                    vol--;
                    set_hw_volume(vol);
                    ESP_LOGI(TAG, "hw volume = %d", vol);
                }
            }
        }
        vol_cnt++;

        /* 播放声音 */
        // i2s_play_sound(&mp3Info, sw);
        i2s_play_sound_ex(sw);


        vTaskDelay(pdMS_TO_TICKS(delay));
    }

    vTaskDelete(NULL);
}

void sound_init(void)
{
    /* 初始化音量 */
    gpio_config_t io_conf = {}; //zero-initialize the config structure.
    io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT; //set as output mode
    io_conf.pin_bit_mask = (1ULL << AMPLIFIER_GAIN_IO); //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = 0;   //disable pull-down mode
    io_conf.pull_up_en = 0; //disable pull-up mode
    ESP_ERROR_CHECK(gpio_config(&io_conf)); //configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_set_level(AMPLIFIER_GAIN_IO, 1));  // 上拉，6dB增益

    /* Initialize i2s peripheral */
    ESP_ERROR_CHECK(i2s_driver_init());
    ESP_ERROR_CHECK(spiffs_init());
    

    ESP_LOGI(TAG, "sound init success!");

    xTaskCreate(i2s_music_task, "i2s_music_task", 40960, NULL, 6, NULL);
}
