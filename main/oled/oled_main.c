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

/* 宏定义 */
#define TEST_TASK_STACK_SIZE    (40960)

#define SCREEN_WIDTH    (128)
#define SCREEN_HEIGHT   (64)
#define SCREEN_PAGE_NUM (8)

/* 结构体定义 */
struct OledMonoBuff {
    int width;
    int height;
    uint8_t *buff;  // buff指针
};

/* 全局变量 */



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
            if ((len2 >= r1 * r1) && (len2 < r2 * r2)) {
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
    double maxR = sqrt(buff->width * buff->width + buff->height * buff->height);    // 对角线长度
    while (outerR <= (maxR + width)) {
        int innerR = (outerR - width) > 0 ? (outerR - width) : 0;
        int midX = buff->width / 2;
        int midY = buff->height / 2;
        draw_circle(buff, midX, midY, innerR, outerR, val);
        outerR += interval;
    }     
}

static struct OledMonoBuff* create_multi_circle_frames(int width, int interval)
{
    int startR = 1;

    struct OledMonoBuff *buffs = (struct OledMonoBuff *)malloc(sizeof(struct OledMonoBuff) * interval);

    for (int i = 0; i < interval; i++) {
        buffs[i].width = 64;
        buffs[i].height = 64;
        int size = SCREEN_WIDTH * SCREEN_HEIGHT / 8;
        buffs[i].buff = (uint8_t *)malloc(size);
        memset(buffs[i].buff, 0, size);
        draw_multi_circle(&buffs[i], startR, width, interval, 1);
        startR++;
    }

    return buffs;
}



static void oled_task(void *arg)
{
    int width = 6;
    int interval = 12;

    struct OledMonoBuff *buffs = create_multi_circle_frames(width, interval);
    ESP_ERROR_CHECK(buffs == NULL);

    OLED_DrawBMP(32, 0, 95, 7, buffs[0].buff);

    while (1) {
        for (int i = 0; i < interval; i++) {
            OLED_DrawBMP(32, 0, 95, 7, buffs[i].buff);
            vTaskDelay(41 / portTICK_PERIOD_MS);
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
    printf("draw ciecle\n");
    xTaskCreate(oled_task, "oled_task", TEST_TASK_STACK_SIZE, NULL, 2, NULL);
}
