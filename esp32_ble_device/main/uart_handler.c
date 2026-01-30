/*
 * UART Handler - Request-Response Implementation
 */

#include <string.h>
#include <stdlib.h>  // For rand() in simulation mode
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "uart_handler.h"
#include <cJSON.h>

static const char *TAG = "UART_HANDLER";

// External variable from gatts_simple.c
extern bool g_connected;

// UART task handle - will be set when task is created
TaskHandle_t uart_task_handle = NULL;

// Global sensor data (thread-safe)
static sensor_data_t g_sensor_data = {
    .temperature = 0.0,
    .humidity = 0.0,
    .led_state = 0,
    .timestamp = 0,
    .valid = false
};

// Initialize UART
esp_err_t uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    return ESP_OK;
}

// Calculate simple XOR checksum
static uint8_t calc_checksum(uint8_t *data, int len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum ^= data[i];
    }
    return sum;
}

// Gửi request sensor data tới STM32
esp_err_t uart_request_sensor_data(void) {
    uint8_t request[4];
    
    request[0] = FRAME_START;           // 0xAA
    request[1] = CMD_REQUEST_DATA;      // 0x01
    request[2] = calc_checksum(&request[1], 1);  // Checksum of CMD
    request[3] = FRAME_END;             // 0x55
    
    int written = uart_write_bytes(UART_NUM, (const char*)request, sizeof(request));
    
    if (written == sizeof(request)) {
        ESP_LOGD(TAG, "Request sent to STM32");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send request");
        return ESP_FAIL;
    }
}

// Gửi lệnh điều khiển LED tới STM32
// Frame: [0xAA][CMD_LED_CONTROL][LED_STATE][CHECKSUM][0x55]
esp_err_t uart_send_led_command(uint8_t led_state) {
    uint8_t command[5];
    
    command[0] = FRAME_START;              // 0xAA
    command[1] = CMD_LED_CONTROL;          // 0x02
    command[2] = led_state;                // 0=OFF, 1=ON
    command[3] = calc_checksum(&command[1], 2);  // Checksum of CMD + LED_STATE
    command[4] = FRAME_END;                // 0x55
    
    ESP_LOGI(TAG, "Sending LED command to STM32: %s", led_state ? "ON" : "OFF");
    ESP_LOG_BUFFER_HEX(TAG, command, 5);
    
    int written = uart_write_bytes(UART_NUM, (const char*)command, sizeof(command));
    
    if (written == sizeof(command)) {
        ESP_LOGI(TAG, "LED command sent successfully");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to send LED command: wrote %d bytes", written);
        return ESP_FAIL;
    }
}


// Parse frame từ STM32 (DHT11 - 8-bit resolution)
// Expected: [0xAA][LEN=3][TEMP][HUM][LED][CHECKSUM][0x55]
// Total: 7 bytes
static bool parse_frame(uint8_t *buf, int len, sensor_data_t *data) {
    ESP_LOGI(TAG, "Parsing buffer: %d bytes", len);
    ESP_LOG_BUFFER_HEX(TAG, buf, len);
    
    // Tìm start byte
    int start_idx = -1;
    for (int i = 0; i < len; i++) {  // Tìm trong toàn bộ buffer
        if (buf[i] == FRAME_START) {
            start_idx = i;
            ESP_LOGI(TAG, "Found START at index %d", i);
            break;
        }
    }
    
    if (start_idx == -1) {
        ESP_LOGW(TAG, "No START byte found (expecting 0x%02X)", FRAME_START);
        ESP_LOGW(TAG, "Buffer first byte: 0x%02X", buf[0]);
        return false;
    }
    
    uint8_t *frame = &buf[start_idx];
    uint8_t data_len = frame[1];
    
    ESP_LOGI(TAG, "Data length: %d", data_len);
    
    // Check có đủ data không (START + LEN + DATA + CHECKSUM + END)
    int required_len = 2 + data_len + 2;  // Total frame size
    int available_len = len - start_idx;
    
    if (available_len < required_len) {
        ESP_LOGW(TAG, "Incomplete frame: need %d bytes, have %d", 
                 required_len, available_len);
        return false;
    }
    
    // Validate checksum
    uint8_t recv_checksum = frame[2 + data_len];
    uint8_t calc_checksum_val = calc_checksum(&frame[1], data_len + 1); // LEN + DATA
    
    ESP_LOGI(TAG, "Checksum: recv=0x%02X calc=0x%02X", recv_checksum, calc_checksum_val);
    
    if (recv_checksum != calc_checksum_val) {
        ESP_LOGW(TAG, "Checksum mismatch: recv=0x%02X calc=0x%02X", 
                 recv_checksum, calc_checksum_val);
        return false;
    }
    
    // Check end byte
    if (frame[3 + data_len] != FRAME_END) {
        ESP_LOGW(TAG, "End byte not found at index %d: 0x%02X", 
                 3 + data_len, frame[3 + data_len]);
        return false;
    }
    
    ESP_LOGI(TAG, "Frame validated - parsing data");
    
    // Parse data: [TEMP][HUM][LED] - DHT11 8-bit values
    if (data_len >= 3) {
        uint8_t temp = frame[2];   // Temperature (0-50°C)
        uint8_t hum = frame[3];    // Humidity (20-90%RH)
        uint8_t led = frame[4];    // LED state (0/1)
        
        data->temperature = (float)temp;
        data->humidity = (float)hum;
        data->led_state = led;
        data->timestamp = esp_timer_get_time();
        data->valid = true;
        
        ESP_LOGI(TAG, "Parsed: %.1f°C %.1f%% LED=%d", 
                 data->temperature, data->humidity, data->led_state);
        return true;
    }
    
    ESP_LOGW(TAG, "Data length too short: %d", data_len);
    return false;
}

// Get latest sensor data (thread-safe)
sensor_data_t get_sensor_data(void) {
    // Check if data is stale (no update for 5 seconds)
    int64_t now = esp_timer_get_time();
    if (g_sensor_data.valid && (now - g_sensor_data.timestamp) > 5000000) {
        ESP_LOGW(TAG, "Sensor data stale (>5s)");
        g_sensor_data.valid = false;
    }
    
    return g_sensor_data;
}


//UART CODE 
void uart_receive_task(void *arg) {
    // Save task handle for LED command notification
    uart_task_handle = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "UART task ready (waiting for Gateway...)");
    
    uint8_t rx_buffer[128];
    sensor_data_t temp_data;
    uint32_t notification_value = 0;
    
    while (1) {
        // Check if Gateway is connected
        if (!g_connected) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        
        // Check for LED command notification (non-blocking, 100ms timeout)
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notification_value, pdMS_TO_TICKS(100)) == pdTRUE) {
            // LED command received from BLE
            if (notification_value & LED_CMD_NOTIFICATION_BIT) {
                uint8_t led_state = notification_value & 0xFF;
                ESP_LOGI(TAG, "[LED CMD INTERRUPT] Processing LED command: %d", led_state);
                
                // Clear buffer before sending LED command
                uart_flush(UART_NUM);
                
                // Send LED command to STM32
                uart_send_led_command(led_state);
                
                // Give STM32 time to process LED command
                vTaskDelay(pdMS_TO_TICKS(500));
                
                // Clear buffer after LED command
                uart_flush(UART_NUM);
                
                // Continue to sensor request after LED is processed
                continue;
            }
        }
        
        // Normal sensor request flow (when no LED command)
        // 1. Gửi request tới STM32
        uart_request_sensor_data();
        
        // Đợi STM32 xử lý (DHT11 needs time)
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // 2. Đợi response (timeout 1500ms - giống Python test)
        int len = uart_read_bytes(UART_NUM, rx_buffer, sizeof(rx_buffer), 
                                  pdMS_TO_TICKS(1500));
        
        if (len > 0) {
            ESP_LOGI(TAG, "Received %d bytes from STM32", len);
            ESP_LOG_BUFFER_HEX(TAG, rx_buffer, len);
            
            // 3. Parse response
            if (parse_frame(rx_buffer, len, &temp_data)) {
                g_sensor_data = temp_data;
                ESP_LOGI(TAG, "[RX] %.1f°C %.1f%% LED=%d", 
                         temp_data.temperature, temp_data.humidity, temp_data.led_state);
            }
        } else if (len == 0) {
            ESP_LOGW(TAG, "[RX] Timeout - No response from STM32");
            g_sensor_data.valid = false;
        } else if (len < 0) {
            ESP_LOGE(TAG, "[RX] UART read error: %d", len);
            g_sensor_data.valid = false;
        }
        
        // 4. Đợi 2 giây trước khi request tiếp
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

