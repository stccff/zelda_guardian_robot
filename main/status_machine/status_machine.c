#include "status_machine.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "status_machine.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
// #include "driver/gpio.h"
// #include "driver/gpio_filter.h"
// #include "hid_host_gamepad.h"
// #include "time.h"

static enum LightStatus g_lightState;
static enum BluetoothStatus g_blStatus;

/**
 * @brief 
 * 
 * @param status 
 */
void sm_set_light_status(enum LightStatus status)
{
    // todo 加锁保护
    g_lightState = status;
    printf("light status = %d\n", status);
    // todo 解锁
}

enum LightStatus sm_get_light_status(void)
{
    return g_lightState;
}

enum BluetoothStatus sm_get_bluetooth_status(void)
{
    return g_blStatus;
}

void sm_set_bluetooth_status(enum BluetoothStatus status)
{
    printf("blutooth status = %d\n", status);
    g_blStatus = status;
}


