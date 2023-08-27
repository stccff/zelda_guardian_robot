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
// #include "minimp3_ex.h"


#define INPUT_BUFFER_SIZE (1024 * 2)

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
struct Mp3 {    // mp3解码缓存
    uint8_t input_buf[INPUT_BUFFER_SIZE];
    int16_t pcm_buf[MINIMP3_MAX_SAMPLES_PER_FRAME];
    uint32_t buffered;
    uint32_t to_read;
    FILE *fp;
    mp3dec_t mp3d;
    mp3dec_frame_info_t info;
};

struct Mp3Node {    // MP3缓存
    int hz;
    int ch;
    int samples;
    int16_t *pcm_buf;
};



static const char *TAG = "sound";
static const char err_reason[][30] = {"input param is invalid",
                                      "operation timeout"
                                     };
static i2s_chan_handle_t g_tx_handle = NULL;
struct Mp3 g_mp3 = {0};

typedef bool (*PlayCtrl)(void);    // 播放控制函数原型



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
 * @return int 
 */
static int read_and_decode_one_frame(void)
{
    // read in the data that is needed to top up the buffer
    size_t n = fread(g_mp3.input_buf + g_mp3.buffered, 1, g_mp3.to_read, g_mp3.fp);
    g_mp3.buffered += n;
    // decode the next frame
    int samples = mp3dec_decode_frame(&g_mp3.mp3d, g_mp3.input_buf, g_mp3.buffered, g_mp3.pcm_buf, &g_mp3.info);
    // we've processed this may bytes from teh buffered data
    g_mp3.buffered -= g_mp3.info.frame_bytes;
    // shift the remaining data to the front of the buffer
    memmove(g_mp3.input_buf, g_mp3.input_buf + g_mp3.info.frame_bytes, g_mp3.buffered);
    // we need to top up the buffer from the file
    g_mp3.to_read = g_mp3.info.frame_bytes;

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
      .max_files = 10,
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

/**
 * @brief 检查是否播放
 */
static bool IsTrue(void)
{
    return true;
}

/**
 * @brief 检查是否播放
 */
static bool IsPlayMusic(void)
{
    static bool status = false;
    const struct XboxData *xbox = get_xbox_pad_data();
    if (xbox->bnt1.btnR == 1) {
        status = true;
    }
    if (xbox->bnt1.btnL == 1) {
        status = false;
    }
    return status;
}

/**
 * @brief 检查是否播放
 */
static bool IsPlaySearch(void)
{
    const struct XboxData *xbox = get_xbox_pad_data();
    return ((xbox->trigRT > (XBOX_TRIGGER_MAX / 20)) && (xbox->trigRT < (XBOX_TRIGGER_MAX / 2))) ? true : false;
}

/**
 * @brief 检查是否播放 
 */
static bool IsPlayLock(void)
{
    const struct XboxData *xbox = get_xbox_pad_data();
    return ((xbox->trigRT > (XBOX_TRIGGER_MAX / 2)) && (xbox->trigRT < (XBOX_TRIGGER_MAX / 20 * 19))) ? true : false;
}

/**
 * @brief 检查是否播放
 */
static bool IsPlayBeam(void)
{
    const struct XboxData *xbox = get_xbox_pad_data();
    return (xbox->trigRT > (XBOX_TRIGGER_MAX / 10 * 9)) ? true : false;
}

/**
 * @brief 重置mp3解码器（从头读取）
 * 
 */
static void ResetMp3Decoder(void)
{
    fseek(g_mp3.fp, 0, SEEK_SET); // 重头读取
    g_mp3.buffered = 0;
    g_mp3.to_read = INPUT_BUFFER_SIZE;   // 初始化变量
    memset(&g_mp3.mp3d, 0, sizeof(mp3dec_t));
    memset(&g_mp3.info, 0, sizeof(mp3dec_frame_info_t));
}

/**
 * @brief 音频播放（单次）
 * 
 */
static void PlayOnece(PlayCtrl ctrlFunc)
{
    esp_err_t ret = ESP_OK;
    int samples = 0;
    do {
        samples = read_and_decode_one_frame();    // 解码
        if (samples <= 0) {
            break;
        }
        /* Write music to speaker */
        size_t size = samples * g_mp3.info.channels * sizeof(short);
        size_t bytes_write;
        ret = i2s_channel_write(g_tx_handle, g_mp3.pcm_buf, size, &bytes_write, portMAX_DELAY);
        if (ret != ESP_OK) {
            /* Since we set timeout to 'portMAX_DELAY' in 'i2s_channel_write' 
            so you won't reach here unless you set other timeout value, if timeout detected, it means write operation failed. */
            ESP_LOGE(TAG, "[music] i2s write failed, %s", err_reason[ret == ESP_ERR_TIMEOUT]);
        }
        if (bytes_write <= 0) {
            ESP_LOGE(TAG, "[music] i2s music play falied.");
        }
    } while (ctrlFunc());
}

/**
 * @brief 音频播放（单曲循环）
 * 
 */
static void PlayLoop(uint32_t loops, PlayCtrl ctrlFunc)
{
    while (ctrlFunc() && loops) {
        PlayOnece(ctrlFunc);
        ResetMp3Decoder();
        loops--;
    }
}


/**
 * @brief 从文件读取mp3，解码，播放（低内存占用）
 * 
 * @param fileName 文件名
 * @param ctrlFunc 是否播放检查函数
 */
static void i2s_play_mp3(char *fileName, uint32_t loops, PlayCtrl ctrlFunc)
{
    esp_err_t ret = ESP_OK;

    g_mp3.fp = fopen(fileName, "r");     // 打开文件
    ESP_ERROR_CHECK(!g_mp3.fp);
    g_mp3.buffered = 0;
    g_mp3.to_read = INPUT_BUFFER_SIZE;   // 初始化变量
    memset(&g_mp3.mp3d, 0, sizeof(mp3dec_t));
    memset(&g_mp3.info, 0, sizeof(mp3dec_frame_info_t));
    mp3dec_init(&g_mp3.mp3d); // mp3 decoder state

    /* 检测文件通道采样率并配置i2s */
    int samples = read_and_decode_one_frame();
    if (samples <=0) {
        ESP_LOGE(TAG, "mp3info detect error");
    }
    ESP_LOGI(TAG, "%s: %d channel, %dHz, bitrate %dkbps",
            fileName, g_mp3.info.channels, g_mp3.info.hz, g_mp3.info.bitrate_kbps);
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(g_mp3.info.hz);
    i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, g_mp3.info.channels);
    ret = i2s_channel_reconfig_std_clock(g_tx_handle, &clk_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_reconfig_std_clock error! ret = %d", ret);
    }
    ret = i2s_channel_reconfig_std_slot(g_tx_handle, &slot_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_reconfig_std_slot error! ret = %d", ret);
    }
    ESP_ERROR_CHECK(i2s_channel_enable(g_tx_handle));   // 使能通道
    ResetMp3Decoder();
    
    /* 播放 */
    PlayLoop(loops, ctrlFunc);

    /* 结束播放 */
    ESP_ERROR_CHECK(i2s_channel_disable(g_tx_handle));
    fclose(g_mp3.fp);
}


// static void i2s_play_sound_ex(bool sw)
// {   
//     esp_err_t ret = ESP_OK;
//     static mp3dec_ex_t dec;
//     // static mp3dec_t mp3d;
//     // static mp3dec_file_info_t info;
//     static mp3d_sample_t *buffer;
//     static size_t readed;
//     static bool lastStatus = false;
//     if (lastStatus != sw) { // 判断开关状态是否切换
//         if (sw == true) {
//             ESP_ERROR_CHECK(i2s_channel_enable(g_tx_handle));   // 使能通道
//             // ret = mp3dec_load(&mp3d, "/spiffs/guardian_beamsightlocking.mp3", &info, NULL, NULL);
//             // if (ret) {
//             //     ESP_LOGE(TAG, "mp3dec_load fail! ret = %d\n", ret);
//             // }
//             /* mp3dec_file_info_t contains decoded samples and info, use free(info.buffer) to deallocate samples */
//             ret = mp3dec_ex_open(&dec, "/spiffs/guardian_beamsightlocking.mp3", MP3D_SEEK_TO_SAMPLE);
//             if (ret) {
//                 ESP_LOGE(TAG, "mp3dec_ex_open fail! ret = %d\n", ret);
//                 return;
//             }
//             /* dec.samples, dec.info.hz, dec.info.layer, dec.info.channels should be filled */
//             ret = mp3dec_ex_seek(&dec, 5936);
//             if (ret) {
//                 ESP_LOGE(TAG, "mp3dec_ex_seek error! ret = %d\n", ret);
//                 mp3dec_ex_close(&dec);
//                 return;
//             }
//             size_t size = dec.samples * sizeof(mp3d_sample_t);
//             buffer = malloc(size);
//             readed = mp3dec_ex_read(&dec, buffer, (17498 - 5936));
// printf("readed = %d, samples = %llu", readed, dec.samples);
//             /* normal eof or error condition */
//             if (readed != dec.samples) {
//                 if (dec.last_error) {
//                     ESP_LOGE(TAG, "last error!\n");
//                 }
//             }
//         } else {
//             ESP_ERROR_CHECK(i2s_channel_disable(g_tx_handle));
//             mp3dec_ex_close(&dec);
//             // free(info.buffer);
//             free(buffer);
//         }
//     }
//     lastStatus = sw;
//     if (sw == false) {
//         return; // 关闭播放直接返回
//     }
//     /* Write music to speaker */
//     size_t bytes_write = 0;
//     ret = i2s_channel_write(g_tx_handle, buffer, readed * sizeof(mp3d_sample_t), &bytes_write, portMAX_DELAY);
//     if (ret != ESP_OK) {
//         /* Since we set timeout to 'portMAX_DELAY' in 'i2s_channel_write'
//         so you won't reach here unless you set other timeout value,
//         if timeout detected, it means write operation failed. */
//         ESP_LOGE(TAG, "[music] i2s write failed, %s", err_reason[ret == ESP_ERR_TIMEOUT]);
//     }
//     if (bytes_write > 0) {
//         // ESP_LOGI(TAG, "[music] i2s music played, %d bytes are written.", bytes_write);
//     } else {
//         ESP_LOGE(TAG, "[music] i2s music play falied.");
//     }
// }

/**
 * @brief 从flash文件读取mp3，解码，并存放在内存中
 * 
 * @param file 文件名
 * @return struct Mp3Node* mp3缓存
 */
static struct Mp3Node* make_mp3_node(char *file)
{   
    esp_err_t ret = ESP_OK;

    /* 初始化mp3解码器 */
    g_mp3.fp = fopen(file, "r");     // 打开文件
    ESP_ERROR_CHECK(!g_mp3.fp);
    g_mp3.buffered = 0;
    g_mp3.to_read = INPUT_BUFFER_SIZE;   // 初始化变量
    memset(&g_mp3.mp3d, 0, sizeof(mp3dec_t));
    memset(&g_mp3.info, 0, sizeof(mp3dec_frame_info_t));
    mp3dec_init(&g_mp3.mp3d); // mp3 decoder state

    /* 检测文件信息 */
    struct Mp3Node *node = heap_caps_malloc(sizeof(struct Mp3Node), MALLOC_CAP_SPIRAM);
    node->samples = 0;
    int samples = 0;
    do {
        samples = read_and_decode_one_frame();    // 解码
        node->samples += samples;
    } while (samples > 0);
    node->ch = g_mp3.info.channels;
    node->hz = g_mp3.info.hz;
    int size = node->samples * node->ch * sizeof(short);
    ESP_LOGI(TAG, "%s: %d samples, %d channel, %dHz, bitrate %dkbps, cost mem %dKB",
            file, node->samples, node->ch, node->hz, g_mp3.info.bitrate_kbps, size / 1024);
    ResetMp3Decoder();
    
    /* 缓存整个数据 */
    node->pcm_buf = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    for (int cur_samples = 0, samples = 0; cur_samples < node->samples; cur_samples += samples) {
        samples = read_and_decode_one_frame();    // 解码
        memcpy(&node->pcm_buf[cur_samples * g_mp3.info.channels], g_mp3.pcm_buf, samples * g_mp3.info.channels * sizeof(short));
    }

    /* 后处理 */
    fclose(g_mp3.fp);

    return node;
}

/**
 * @brief 释放缓存的pcm音频
 * 
 * @param node 
 */
static void free_mp3_node(struct Mp3Node *node)
{   
    free(node->pcm_buf);
    free(node);
}

/**
 * @brief 播放已经解码缓存的mp3 pcm音频（低延迟）
 * 
 * @param node 已解析好的mp3节点
 * @param startSample 起始样本
 * @param endSample 结束样本
 * @param loops 循环次数（最大INT_MAX）
 * @param ctrlFunc 播放控制函数
 */
static void i2s_play_mp3_node(struct Mp3Node *node, int startSample, int endSample, uint32_t loops, PlayCtrl ctrlFunc)
{
    esp_err_t ret = ESP_OK;
    
    /* 设置驱动 */
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(node->hz);
    i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, node->ch);
    ret = i2s_channel_reconfig_std_clock(g_tx_handle, &clk_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_reconfig_std_clock error! ret = %d", ret);
    }
    ret = i2s_channel_reconfig_std_slot(g_tx_handle, &slot_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_reconfig_std_slot error! ret = %d", ret);
    }
    ESP_ERROR_CHECK(i2s_channel_enable(g_tx_handle));   // 使能通道

    /* 播放声音 */
    while (ctrlFunc() && loops) {
        /* Write music to speaker */
        size_t bytes_write;
        size_t startIndex = startSample * node->ch;
        size_t len;
        if (endSample > node->samples) {
            len = (node->samples - startSample) * node->ch * sizeof(short);
        } else  {
            len =  (endSample - startSample) * node->ch * sizeof(short);
        }
        ret = i2s_channel_write(g_tx_handle, &node->pcm_buf[startIndex], len, &bytes_write, portMAX_DELAY);
        if (ret != ESP_OK) {
            /* Since we set timeout to 'portMAX_DELAY' in 'i2s_channel_write' 
            so you won't reach here unless you set other timeout value, if timeout detected, it means write operation failed. */
            ESP_LOGE(TAG, "[music] i2s write failed, %s", err_reason[ret == ESP_ERR_TIMEOUT]);
        }
        if (bytes_write <= 0) {
            ESP_LOGE(TAG, "[music] i2s music play falied.");
        }
        loops--;
    }

    /* 结束播放 */
    ESP_ERROR_CHECK(i2s_channel_disable(g_tx_handle));
}



/**
 * @brief 声音播放任务
 * 
 * @param args 
 */
static void i2s_music_task(void *args)
{
    struct Mp3Node *node[3];
    node[0] = make_mp3_node("/spiffs/guardian_beamsightsearch.mp3");
    node[1] = make_mp3_node("/spiffs/guardian_beamsightlocking.mp3");
    node[2] = make_mp3_node("/spiffs/SE_Guardian_Beam02.mp3");

    uint32_t vol = 0;
    while (1) {
        const struct XboxData *xbox = get_xbox_pad_data();
        // bool sw = (xbox->trigRT > (XBOX_TRIGGER_MAX / 2) ? true : false);

        /* 设置音量 */
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

        /* 播放声音 */
        if (IsPlaySearch() || IsPlayLock() || IsPlayBeam()) {
            i2s_play_mp3_node(node[0], 0, 5115, INT32_MAX, IsPlaySearch);
            i2s_play_mp3_node(node[1], 5936, 17498, INT32_MAX, IsPlayLock);
            i2s_play_mp3_node(node[2], 0, INT32_MAX, INT32_MAX, IsPlayBeam);
            // i2s_play_mp3("/spiffs/guardian_beamsightsearch.mp3", true, IsPlay);
        }
        
        if (IsPlayMusic()) {
            i2s_play_mp3("/spiffs/guardian_battle.mp3", 2, IsPlayMusic);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelete(NULL);
}

/**
 * @brief 音频初始化
 * 
 */
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
