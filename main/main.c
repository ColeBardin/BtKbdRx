/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "driver/gpio.h"

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

#define DBA_TARGET "6c:93:08:61:00:28"
#define RX_LED GPIO_NUM_10

static const char *TAG = "BtKbdRx";

bool bt_rx;
bool target_paired;

void configure_gpio() {
    // Configure GPIO12 as a digital output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RX_LED),     // Select for RX LED
        .mode = GPIO_MODE_OUTPUT,             // Set as output mode
        .pull_up_en = GPIO_PULLUP_DISABLE,    // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,// Disable pull-down
        .intr_type = GPIO_INTR_DISABLE        // Disable interrupts
    };

    // Apply the configuration
    gpio_config(&io_conf);
}

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: { // pair / connect
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            //esp_hidh_dev_dump(param->open.dev, stdout);
            target_paired = true;
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
    case ESP_HIDH_INPUT_EVENT: { // received shit
        bt_rx = true;
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
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

void bt_rx_task(void *pvParameters){
    for(;;){
        if(bt_rx){
            bt_rx = false;
            gpio_set_level(RX_LED, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(RX_LED, 0);
        }else{
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}

#define SCAN_DURATION_SECONDS 2

void hid_pair_task(void *pvParameters)
{
    char found_dba_str[18];

    while(!target_paired){
        size_t results_len = 0;
        esp_hid_scan_result_t *results = NULL;
        // pairing
        ESP_LOGI(TAG, "SCAN...");
        esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
        ESP_LOGI(TAG, "SCAN: %u results", results_len);
        if (results_len) {
            esp_hid_scan_result_t *r = results;
            esp_hid_scan_result_t *cr = NULL;
            while (r) {
                snprintf(found_dba_str, sizeof(found_dba_str), ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(r->bda)); 
                if(!strncmp(DBA_TARGET, found_dba_str, strlen(DBA_TARGET))){
                    printf("Target found in pairing:\n");
                    printf(" %s: %s ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", found_dba_str);
                    printf("RSSI: %d, ", r->rssi);
                    printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
                    if (r->transport == ESP_HID_TRANSPORT_BLE) {
                        cr = r;
                        //printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                        //printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
                    }
#endif // CONFIG_BT_BLE_ENABLED
#if CONFIG_BT_HID_HOST_ENABLED
                    if (r->transport == ESP_HID_TRANSPORT_BT) {
                        cr = r;
                        //printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                        //esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                        //printf("] srv 0x%03x, ", r->bt.cod.service);
                        //print_uuid(&r->bt.uuid);
                        //printf(", ");
                    }
#endif // CONFIG_BT_HID_HOST_ENABLED
                    printf("NAME: %s ", r->name ? r->name : "");
                    printf("\n");
                    break; // found target, stop checking others
                }
                r = r->next;
            }
            // attempt to configure if target found
            if (cr) {
                //open the last result
                printf("Opening target device...\n");
                esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
            }
            //free the results
            esp_hid_scan_results_free(results);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    configure_gpio();
    
    char bda_str[18] = {0};
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

    ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

    target_paired = false;
    bt_rx = false;
    xTaskCreate(&hid_pair_task, "hid_pair_task", 6 * 1024, NULL, 3, NULL);
    xTaskCreate(&bt_rx_task, "bt_rx_task", 2 * 1024, NULL, 2, NULL);
}

