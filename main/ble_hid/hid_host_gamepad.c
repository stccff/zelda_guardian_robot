/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_hidh.h"
#include "esp_hid_gap.h"
#include "hid_host_gamepad.h"
#include "status_machine.h"


#define SCAN_DURATION_SECONDS 5
#define TASK_PERIOD 100 // ms

/* 全局变量 */
static const char *TAG = "ESP_HIDH_DEMO";

static struct XboxData g_xboxData = {   // 初始化摇杆数据
    .joyLx = 0xffff / 2,
    .joyLy = 0xffff / 2,
    .joyRx = 0xffff / 2,
    .joyRy = 0xffff / 2,
    .trigLT = 0,
    .trigRT = 0,
};



/**
 * @brief 手柄按键数据解析
 * 
 * @param input 数据字节流
 */
static void parsing_key(uint8_t *input)
{
    memcpy(&g_xboxData, input, sizeof(struct XboxData));
}

/**
 * @brief Get the xbox pad data object
 * 
 * @return const struct XboxData* 
 */
const struct XboxData* get_xbox_pad_data(void)
{
    return &g_xboxData;
}



static void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
        } else {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
        static uint8_t lastData[64];
        if (memcmp(lastData, param->input.data, param->input.length)) {
            memcpy(lastData, param->input.data, param->input.length);
        } else {
            break;
        }
        parsing_key(param->input.data);

        // const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        // ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        // ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}

void bt_hid_task(void *pvParameters)
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;

    while (1) {
        if (sm_get_bluetooth_status() == SM_BL_SCAN_OFF) {
            vTaskDelay(TASK_PERIOD / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "SCAN...");
        //start scan for HID devices
        esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
        ESP_LOGI(TAG, "SCAN: %u results", results_len);

        if (results_len) {
            sm_set_bluetooth_status(SM_BL_SCAN_OFF);
            esp_hid_scan_result_t *r = results;
            esp_hid_scan_result_t *cr = NULL;
            while (r) {
                printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
                printf("RSSI: %d, ", r->rssi);
                printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
                if (r->transport == ESP_HID_TRANSPORT_BLE) {
                    cr = r;
                    printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                    printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
                }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
                if (r->transport == ESP_HID_TRANSPORT_BT) {
                    cr = r;
                    printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                    esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                    printf("] srv 0x%03x, ", r->bt.cod.service);
                    print_uuid(&r->bt.uuid);
                    printf(", ");
                }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
                printf("NAME: %s ", r->name ? r->name : "");
                printf("\n");
                r = r->next;
            }
            if (cr) {
                //open the last result
                esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
            }
            //free the results
            esp_hid_scan_results_free(results);
        }
    };

    // printf("%s, line = %d\n", __FUNCTION__, __LINE__);
    vTaskDelete(NULL);
}

void hid_host_init(void)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    esp_err_t ret;
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK( esp_hidh_init(&config) );

    xTaskCreate(&bt_hid_task, "bt_hid_task", 6 * 1024, NULL, 15, NULL);
}
