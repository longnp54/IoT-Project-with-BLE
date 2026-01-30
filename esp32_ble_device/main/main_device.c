/*
 * ESP32 BLE Device - Environmental Sensor
 * SIMPLE VERSION: UART RX + BLE GATT Server with Notification
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "driver/gpio.h"
#include "uart_handler.h"

#define TAG "BLE_DEVICE"

// LED GPIO
#define LED_GPIO  GPIO_NUM_2 // Built-in LED trên hầu hết ESP32 board

// UUIDs
#define SERVICE_UUID 0x00FF
#define CHAR_NOTIFY_UUID 0xFF01  // Sensor data notification
#define CHAR_WRITE_UUID  0xFF02  // LED control command

// BLE State
static uint16_t g_conn_id = 0;
static uint16_t g_service_handle = 0;     // Service handle
static uint16_t g_char_handle = 0;        // NOTIFY characteristic handle
static uint16_t g_cccd_handle = 0;        // CCCD descriptor handle (QUAN TRỌNG)
static uint16_t g_write_char_handle = 0;  // WRITE characteristic handle
static esp_gatt_if_t g_gatts_if = ESP_GATT_IF_NONE;
bool g_connected = false;  // Exported to uart_handler.c
static bool g_notify_enabled = false;

// Advertising data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GAP Event Handler
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "[BLE] Advertising... (waiting for Gateway)");
        }
        break;
    default:
        break;
    }
}

// GATTS Event Handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        g_gatts_if = gatts_if;
        esp_ble_gap_set_device_name("ENV_SENSOR");
        esp_ble_gap_config_adv_data(&adv_data);
        
        // Create service
        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,
            .id.inst_id = 0,
            .id.uuid.len = ESP_UUID_LEN_16,
            .id.uuid.uuid.uuid16 = SERVICE_UUID,
        };
        esp_ble_gatts_create_service(gatts_if, &service_id, 8);  // Tăng từ 4 lên 8
        break;
    
    case ESP_GATTS_CREATE_EVT:
        g_service_handle = param->create.service_handle;
        ESP_LOGI(TAG, "Service created, handle: 0x%04X", g_service_handle);
        
        // Add NOTIFY characteristic (0xFF01) - Sensor data
        // NOTE: Chỉ add NOTIFY trước, WRITE sẽ add sau trong callback
        esp_bt_uuid_t notify_char_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid.uuid16 = CHAR_NOTIFY_UUID,
        };
        esp_gatt_char_prop_t notify_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_ble_gatts_add_char(g_service_handle, &notify_char_uuid,
                              ESP_GATT_PERM_READ,
                              notify_property, NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.char_uuid.uuid.uuid16 == CHAR_NOTIFY_UUID) {
            g_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "NOTIFY Characteristic added, handle: 0x%04X", g_char_handle);
            
            // Add descriptor (CCCD for notification)
            esp_bt_uuid_t descr_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
            };
            
            esp_ble_gatts_add_char_descr(param->add_char.service_handle, 
                                         &descr_uuid,
                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 
                                         NULL, NULL);
        } else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_WRITE_UUID) {
            g_write_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "WRITE Characteristic added, handle: 0x%04X", g_write_char_handle);
            
            // Start service after ALL characteristics and descriptors are added
            esp_err_t ret = esp_ble_gatts_start_service(g_service_handle);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✓ Service 0x%04X STARTED successfully!", SERVICE_UUID);
            } else {
                ESP_LOGE(TAG, "Failed to start service: %s", esp_err_to_name(ret));
            }
        }
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if (param->add_char_descr.status == ESP_GATT_OK) {
            g_cccd_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "CCCD Descriptor added, handle: 0x%04X", g_cccd_handle);
            
            // Now add WRITE characteristic (0xFF02) - LED control
            ESP_LOGI(TAG, "Adding WRITE characteristic...");
            esp_bt_uuid_t write_char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = CHAR_WRITE_UUID,
            };
            esp_gatt_char_prop_t write_property = ESP_GATT_CHAR_PROP_BIT_WRITE;
            esp_ble_gatts_add_char(g_service_handle, &write_char_uuid,
                                  ESP_GATT_PERM_WRITE,
                                  write_property, NULL, NULL);
        } else {
            ESP_LOGE(TAG, "Failed to add CCCD descriptor, status: %d", param->add_char_descr.status);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "[CONNECTED] Gateway conn_id=%d", param->connect.conn_id);
        g_conn_id = param->connect.conn_id;
        g_connected = true;
        gpio_set_level(LED_GPIO, 1);  // Bật LED
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "[DISCONNECTED] Gateway disconnected");
        g_connected = false;
        g_notify_enabled = false;
        gpio_set_level(LED_GPIO, 0);  // Tắt LED
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_READ_EVT: {
        sensor_data_t sensor = get_sensor_data();
        
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(rsp));
        rsp.attr_value.handle = param->read.handle;
        
        int16_t temp = (int16_t)(sensor.temperature * 100);
        uint16_t hum = (uint16_t)(sensor.humidity * 100);
        
        rsp.attr_value.value[0] = (temp >> 8) & 0xFF;
        rsp.attr_value.value[1] = temp & 0xFF;
        rsp.attr_value.value[2] = (hum >> 8) & 0xFF;
        rsp.attr_value.value[3] = hum & 0xFF;
        rsp.attr_value.value[4] = sensor.led_state;
        rsp.attr_value.value[5] = sensor.valid ? 1 : 0;
        rsp.attr_value.len = 6;
        
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT:
        // Kiểm tra WRITE vào WRITE characteristic (LED control)
        if (param->write.handle == g_write_char_handle && param->write.len == 1) {
            uint8_t led_cmd = param->write.value[0];
            ESP_LOGI(TAG, "[LED] Received command: %d (%s)", 
                     led_cmd, led_cmd ? "ON" : "OFF");
            
            // Enqueue LED command to UART command pipeline
            // This ensures proper synchronization without blocking
            esp_err_t ret = uart_enqueue_led_command(led_cmd);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "[LED] Failed to enqueue command");
            }
            
            // Send BLE response immediately (command is queued, not executed yet)
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                       param->write.trans_id, ESP_GATT_OK, NULL);
        }
        // Kiểm tra WRITE vào CCCD (enable/disable notification)
        else if (param->write.len == 2) {
            uint16_t value = param->write.value[0] | (param->write.value[1] << 8);
            if (value == 0x0001) {
                g_notify_enabled = true;
                ESP_LOGI(TAG, "[NOTIFY] Enabled");
            } else if (value == 0x0000) {
                g_notify_enabled = false;
                ESP_LOGI(TAG, "[NOTIFY] Disabled");
            }
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                           param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        break;

    default:
        break;
    }
}

// Task: Send notifications every 2 seconds
void notify_task(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        if (g_connected && g_notify_enabled && g_char_handle != 0) {
            sensor_data_t sensor = get_sensor_data();
            
            uint8_t data[6];
            int16_t temp = (int16_t)(sensor.temperature * 100);
            uint16_t hum = (uint16_t)(sensor.humidity * 100);
            
            data[0] = (temp >> 8) & 0xFF;
            data[1] = temp & 0xFF;
            data[2] = (hum >> 8) & 0xFF;
            data[3] = hum & 0xFF;
            data[4] = sensor.led_state;
            data[5] = sensor.valid ? 1 : 0;  // Gửi cả khi invalid
            
            esp_ble_gatts_send_indicate(g_gatts_if, g_conn_id, g_char_handle, 6, data, false);
            
            if (sensor.valid) {
                ESP_LOGI(TAG, "[TX] %.1f°C %.1f%% LED=%d", sensor.temperature, sensor.humidity, sensor.led_state);
            } else {
                ESP_LOGW(TAG, "[TX] Invalid (no STM32 response)");
            }
        }
    }
}

// Periodic sensor request task
void sensor_request_task(void *arg) {
    ESP_LOGI(TAG, "[SENSOR_REQ] Task started");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(3000));  // Request every 3 seconds
        
        if (g_connected) {
            // Enqueue sensor request
            esp_err_t ret = uart_enqueue_sensor_request();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "[SENSOR_REQ] Failed to enqueue");
            }
        }
    }
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);
    
    ESP_LOGI(TAG, "ESP32 Environmental Sensor Device Starting...");
    
    // Khởi tạo LED
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO, 0);  // Tắt LED ban đầu
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));
    
    ESP_ERROR_CHECK(uart_init());
    
    // Create tasks with queue-based architecture
    xTaskCreate(uart_executor_task, "uart_exec", 4096, NULL, 10, NULL);
    xTaskCreate(sensor_request_task, "sensor_req", 2048, NULL, 5, NULL);
    xTaskCreate(notify_task, "notify", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "System Ready!");
}
