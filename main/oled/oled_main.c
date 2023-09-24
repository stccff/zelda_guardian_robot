/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "oled_drv.h"
#include "driver/uart.h"
#include "picture.h"
#include <math.h>
#include "oled_main.h"
#include "status_machine.h"
#include "esp_log.h"
#include "pwr_ctrl.h"

/* 宏定义 */
#define TEST_TASK_STACK_SIZE    (10 * 1024)

#define SCREEN_WIDTH    (128)   // 屏幕长
#define SCREEN_HEIGHT   (64)    // 屏幕高
#define SCREEN_PAGE_NUM (8)     // 屏幕页数

#define CIRCLE_WIDTH    (48)    // 画圆的buffer长
#define CIRCLE_HEIGHT   (48)    // 画圆的buffer宽
#define CIRCLE_x        (21)    // 圆心在buff中的x坐标
#define CIRCLE_y        (24)    // 圆心在buff中的y坐标
#define MASK_R          (21)    // 外部遮罩半径

/* 结构体定义 */
struct OledMonoBuff {
    int width;
    int height;
    uint8_t *buff;  // buff指针
};

/* 全局变量 */
static const char *TAG = "OLED";



static double square(double x)
{
	return x * x;
}

/**
 * @brief 设置一个Byte中对应的bit像素点
 * 
 * @param pixel8 一个子节对应的8个像素
 * @param index 设置的像素索引
 * @param val 像素颜色（黑/白）
 */
static void set_bit(unsigned char *pixel8, int index, int val)
{
    if (val == 0) {
        *pixel8 &= ~((unsigned char)(1 << index));
    } else {
        *pixel8 |= (unsigned char)(1 << index);
    }
}

/**
 * @brief 设置一个绘制buff中的一个像素值
 * 
 * @param buff 屏幕buff结构体指针
 * @param row 设置的行数
 * @param col 设置的列数
 * @param val 像素颜色（黑/白）
 */
static void oled12864_set_pixel(struct OledMonoBuff *buff, int row, int col, int val)
{
    if ((row >= buff->height) || (col >= buff->width)) {
        return;
    }
    
    int page = col / 8;
    int index = col % 8;
    set_bit(&buff->buff[page * buff->width + row], index, val);
}

/**
 * @brief 绘制一个环形，存放在指定buff中
 * 
 * @param buff 屏幕buff结构体指针
 * @param x 圆心横坐标
 * @param y 圆心纵坐标
 * @param r1 内径
 * @param r2 外径
 * @param val 像素颜色（黑/白）
 */
static void draw_circle(struct OledMonoBuff *buff, int x, int y, int r1, int r2, int val)
{
    for (int i = 0; i < buff->height; i++) {
        for (int j = 0; j < buff->width; j++) {
			double len2 = square(i - x) + square(j - y);
            if ((len2 >= r1 * r1) && (len2 < r2 * r2) && (len2 <= MASK_R * MASK_R)) {
                oled12864_set_pixel(buff, i, j, val);
            }
        }
    }
}

/**
 * @brief 绘制多重环形图案，并写入指定的buff中
 * 
 * @param buff 屏幕buff结构体指针
 * @param outerR 最内部环形外径
 * @param width 环形宽度（内外径差）
 * @param interval 相邻环形半径
 * @param val 像素颜色（黑/白）
 */
static void draw_multi_circle(struct OledMonoBuff *buff, int outerR, int width, int interval, int val)
{
    // width = 6;
    double maxR = sqrt(buff->width * buff->width + buff->height * buff->height);    // 绘制圆的最大半径：使用对角线长度，填充整个buff
    while (outerR <= (maxR + width + 1)) {  // 绘制圆直至内径到buff边缘
        int innerR = (outerR - width) > 0 ? (outerR - width) : 0;
        int midX = CIRCLE_x;
        int midY = CIRCLE_y;
        draw_circle(buff, midX, midY, innerR, outerR, val);
        outerR += interval;
    }     
}

/**
 * @brief 创建多重圆环扩散帧（守护者眼部动画帧）
 * 
 * @param width 圆环宽度
 * @param interval 两个圆环间隔
 * @return struct OledMonoBuff* 
 */
static struct OledMonoBuff* create_multi_circle_frames(int width, int interval)
{
    int startR = 1;

    struct OledMonoBuff *buffs = (struct OledMonoBuff *)malloc(sizeof(struct OledMonoBuff) * interval);

    for (int i = 0; i < interval; i++) {
        buffs[i].width = CIRCLE_WIDTH;      // buff长
        buffs[i].height = CIRCLE_HEIGHT;    // buff高
        int size = CIRCLE_WIDTH * CIRCLE_HEIGHT / 8;    // 一帧的大小
        buffs[i].buff = (uint8_t *)malloc(size);
        memset(buffs[i].buff, 0, size);
        draw_multi_circle(&buffs[i], startR, width, interval, 1);
        startR++;
    }

    return buffs;
}



static void oled_task(void *arg)
{
    int interval = 12;
    struct OledMonoBuff *normalBuffs = create_multi_circle_frames(6, interval);
    ESP_ERROR_CHECK(normalBuffs == NULL);

    ESP_LOGI(TAG, "draw circle");


    while (1) {
        if (sm_get_light_status() <= SM_NO_LIGHT) {
            OLED_Display_Off();
            vTaskDelay(500 / portTICK_PERIOD_MS);
            continue;
        } else {
            OLED_Display_On();
        }

        switch (oled_get_display()) {
            case OLED_CIRCLE:
                for (int i = 0; i < interval; i++) {
                    OLED_DrawBMP(0, 2, 47, 7, normalBuffs[i].buff);
                    vTaskDelay(41 / portTICK_PERIOD_MS);
                }
                break;
            case OLED_BAT:
                OLED_Clear();
                float percent;
                int vol = get_bat_voltage(&percent);

                OLED_ShowChar90(20, 3, 'B');
                OLED_ShowChar90(20, 4, 'a');
                OLED_ShowChar90(20, 5, 't');
                OLED_ShowChar90(20, 6, ':');
                char vones = vol / 1000;
                char tenths = (vol / 100) % 10;
                char hundredths = (vol / 10) % 10;
                OLED_ShowChar90(4, 3, '0' + vones);
                OLED_ShowChar90(4, 4, '.');
                OLED_ShowChar90(4, 5, '0' + tenths);
                OLED_ShowChar90(4, 6, '0' + hundredths);
                vTaskDelay(2000 / portTICK_PERIOD_MS);

                OLED_Clear();
                OLED_ShowChar90(20, 3, 'B');
                OLED_ShowChar90(20, 4, 'a');
                OLED_ShowChar90(20, 5, 't');
                OLED_ShowChar90(20, 6, ':');
                char ones = (int)(percent * 100) % 10;
                char tens = (int)(percent * 100) / 10 % 10;
                char hundruds = (int)(percent * 100) / 100 % 10;
                if (hundruds) { // 百位为1
                    OLED_ShowChar90(4, 3, '0' + hundruds);
                }
                if (tens || hundruds) {
                    OLED_ShowChar90(4, 4, '0' + tens);
                }
                OLED_ShowChar90(4, 5, '0' + ones);
                OLED_ShowChar90(4, 6, '%');
                vTaskDelay(3000 / portTICK_PERIOD_MS);

                OLED_Clear();
                oled_set_display(OLED_CIRCLE);
                break;
            case OLED_ATTACK:
                int frameSize = CIRCLE_WIDTH * CIRCLE_HEIGHT / 8;    // 一帧的大小
                uint8_t *buff = malloc(frameSize);
                memset(buff, 0xff, frameSize);
                // draw_circle(buff, CIRCLE_x, CIRCLE_y, 0, MASK_R, 1);
                OLED_DrawBMP(0, 2, 47, 7, buff);
                vTaskDelay(600 / portTICK_PERIOD_MS);

                oled_set_display(OLED_CIRCLE);
            default:
                break;
        }
    }
}

void oled_main(void)
{
    // esp_err_t ret;
    OLED_Init();// 初始化oled

    // OLED_ShowString(0, 0, "hello world");
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    // OLED_ShowString(0, 0, "           ");   // 清除hello world
    // OLED_DrawBMP(32, 0, 95, 7, LOGO);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    OLED_Clear();

    xTaskCreate(oled_task, "oled_task", TEST_TASK_STACK_SIZE, NULL, 10, NULL);
}
