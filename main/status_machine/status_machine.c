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
static SemaphoreHandle_t g_lightMuxHandle = NULL;
static SemaphoreHandle_t g_blMuxHandle = NULL;


SemaphoreHandle_t xSemaphore = NULL;

/* A task that creates a semaphore. */
void vATask( void * pvParameters )
{
    /* Create the semaphore to guard a shared resource.  As we are using
    the semaphore for mutual exclusion we create a mutex semaphore
    rather than a binary semaphore. */
    xSemaphore = xSemaphoreCreateMutex();
}

/* A task that uses the semaphore. */
void vAnotherTask( void * pvParameters )
{
    /* ... Do other things. */

    if( xSemaphore != NULL )
    {
        /* See if we can obtain the semaphore.  If the semaphore is not
        available wait 10 ticks to see if it becomes free. */
        if( xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ) == pdTRUE )
        {
            /* We were able to obtain the semaphore and can now access the
            shared resource. */

            /* ... */

            /* We have finished accessing the shared resource.  Release the
            semaphore. */
            xSemaphoreGive( xSemaphore );
        }
        else
        {
            /* We could not obtain the semaphore and can therefore not access
            the shared resource safely. */
        }
    }
}

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

