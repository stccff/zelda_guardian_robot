#include "status_machine.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"


/* 全局变量 */
static const char *TAG = "SM";
static enum LightStatus g_lightState;
static enum BluetoothStatus g_blStatus;
static enum OledDisplayType g_oledType;

static SemaphoreHandle_t g_lightMuxHandle = NULL;
static SemaphoreHandle_t g_blMuxHandle = NULL;
static SemaphoreHandle_t g_oledMuxHandle = NULL;

/**
 * @brief 状态机初始化
 * 
 */
void status_machine_init(void)
{
    /* MuxSem */
    g_lightMuxHandle = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(!g_lightMuxHandle);

    g_blMuxHandle = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(!g_lightMuxHandle);

    g_oledMuxHandle = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(!g_oledMuxHandle); 
}

/**
 * @brief 设置灯光状态机
 * 
 * @param status 
 */
void sm_set_light_status(enum LightStatus status)
{
    ESP_ERROR_CHECK(!g_lightMuxHandle);

    xSemaphoreTake(g_lightMuxHandle, portMAX_DELAY);    // 获取互斥量
    g_lightState = status;
    xSemaphoreGive(g_lightMuxHandle); // 释放互斥量

    ESP_LOGI(TAG, "light status = %d", g_lightState);
}

/**
 * @brief 获取灯光状态
 * 
 * @return enum LightStatus 
 */
enum LightStatus sm_get_light_status(void)
{
    return g_lightState;
}

/**
 * @brief 设置蓝牙状态机
 * 
 * @param status 
 */
void sm_set_bluetooth_status(enum BluetoothStatus status)
{
    ESP_ERROR_CHECK(!g_blMuxHandle);

    xSemaphoreTake(g_blMuxHandle, portMAX_DELAY);    // 获取互斥量
    g_blStatus = status;
    xSemaphoreGive(g_blMuxHandle); // 释放互斥量

    ESP_LOGI(TAG, "blutooth status = %d", g_blStatus);
}

/**
 * @brief 获取蓝牙状态
 * 
 * @param status 
 */
enum BluetoothStatus sm_get_bluetooth_status(void)
{
    return g_blStatus;
}

/**
 * @brief 设置oled显示内容
 * 
 * @param type 
 */
void oled_set_display(enum OledDisplayType type)
{
    ESP_ERROR_CHECK(!g_oledMuxHandle);

    xSemaphoreTake(g_oledMuxHandle, portMAX_DELAY);    // 获取互斥量
    g_oledType = type;
    xSemaphoreGive(g_oledMuxHandle); // 释放互斥量

    ESP_LOGI(TAG, "oled display type = %d", g_oledType);
}

/**
 * @brief 获取oled显示内容
 * 
 * @return enum OledDisplayType 
 */
enum OledDisplayType oled_get_display(void)
{
    return g_oledType;
}



