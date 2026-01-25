/*
 * UART Handler - Request-Response Implementation
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "uart_handler.h"

static const char *TAG = "UART_HANDLER";

// External variable from gatts_simple.c
extern bool g_connected;

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

// Parse frame từ STM32
// Expected: [0xAA][LEN=5][TEMP_H][TEMP_L][HUM_H][HUM_L][LED][CHECKSUM][0x55]
static bool parse_frame(uint8_t *buf, int len, sensor_data_t *data) {
    // Tìm start byte
    int start_idx = -1;
    for (int i = 0; i < len - 8; i++) {
        if (buf[i] == FRAME_START) {
            start_idx = i;
            break;
        }
    }
    
    if (start_idx == -1) {
        return false;
    }
    
    uint8_t *frame = &buf[start_idx];
    uint8_t data_len = frame[1];
    
    // Check có đủ data không (START + LEN + DATA + CHECKSUM + END)
    if (start_idx + 2 + data_len + 2 > len) {
        ESP_LOGW(TAG, "Incomplete frame");
        return false;
    }
    
    // Validate checksum
    uint8_t recv_checksum = frame[2 + data_len];
    uint8_t calc_checksum_val = calc_checksum(&frame[1], data_len + 1); // LEN + DATA
    
    if (recv_checksum != calc_checksum_val) {
        ESP_LOGW(TAG, "Checksum mismatch: recv=0x%02X calc=0x%02X", 
                 recv_checksum, calc_checksum_val);
        return false;
    }
    
    // Check end byte
    if (frame[3 + data_len] != FRAME_END) {
        ESP_LOGW(TAG, "End byte not found");
        return false;
    }
    
    // Parse data: [TEMP_H][TEMP_L][HUM_H][HUM_L][LED]
    if (data_len >= 5) {
        int16_t temp_raw = (frame[2] << 8) | frame[3];
        uint16_t hum_raw = (frame[4] << 8) | frame[5];
        uint8_t led = frame[6];
        
        data->temperature = temp_raw / 100.0f;
        data->humidity = hum_raw / 100.0f;
        data->led_state = led;
        data->timestamp = esp_timer_get_time();
        data->valid = true;
        return true;
    }
    
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

// UART Receive Task - Request-Response mode
void uart_receive_task(void *arg) {
    ESP_LOGI(TAG, "UART task ready (waiting for Gateway...)");
    
    uint8_t rx_buffer[128];
    sensor_data_t temp_data;
    
    while (1) {
        // Check if Gateway is connected
        if (!g_connected) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        
        // 1. Gửi request tới STM32
        uart_request_sensor_data();
        
        // 2. Đợi response (timeout 200ms)
        int len = uart_read_bytes(UART_NUM, rx_buffer, sizeof(rx_buffer), 
                                  pdMS_TO_TICKS(200));
        
        if (len > 0) {
            ESP_LOGD(TAG, "Received %d bytes from STM32", len);
            ESP_LOG_BUFFER_HEXDUMP(TAG, rx_buffer, len, ESP_LOG_DEBUG);
            
            // 3. Parse response
            if (parse_frame(rx_buffer, len, &temp_data)) {
                g_sensor_data = temp_data;
                ESP_LOGI(TAG, "[RX] %.1f°C %.1f%% LED=%d", 
                         temp_data.temperature, temp_data.humidity, temp_data.led_state);
            }
        } else if (len == 0) {
            ESP_LOGW(TAG, "[RX] Timeout");
            g_sensor_data.valid = false;
        }
        
        // 4. Đợi 2 giây trước khi request tiếp
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
