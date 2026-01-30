/*
 * UART Handler - Queue-based Command Pipeline with ACK/NACK
 * Implements proper synchronization and recovery mechanisms
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_gatts_api.h"
#include "uart_handler.h"

static const char *TAG = "UART_HANDLER";

// External variable from main_device.c
extern bool g_connected;

// Queue handle for command pipeline
QueueHandle_t uart_cmd_queue = NULL;

// Global sensor data (protected by mutex conceptually - simplified for now)
static sensor_data_t g_sensor_data = {
    .temperature = 0.0,
    .humidity = 0.0,
    .led_state = 0,
    .timestamp = 0,
    .valid = false
};

// UART RX buffer for framing
#define RX_BUFFER_SIZE 256
static uint8_t uart_rx_buffer[RX_BUFFER_SIZE];
static int uart_rx_index = 0;

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
    
    // Create command queue
    uart_cmd_queue = xQueueCreate(UART_CMD_QUEUE_SIZE, sizeof(uart_cmd_t));
    if (uart_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create command queue");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Enqueue sensor request command
esp_err_t uart_enqueue_sensor_request(void) {
    if (uart_cmd_queue == NULL) {
        return ESP_FAIL;
    }
    
    uart_cmd_t cmd = {
        .type = CMD_TYPE_SENSOR_REQUEST,
        .data_len = 0,
        .timeout_ms = UART_CMD_TIMEOUT_MS,
        .retry_count = 0
    };
    
    if (xQueueSend(uart_cmd_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
        ESP_LOGD(TAG, "Sensor request enqueued");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Failed to enqueue sensor request (queue full)");
        return ESP_FAIL;
    }
}

// Enqueue LED control command (HIGH PRIORITY - add to front)
esp_err_t uart_enqueue_led_command(uint8_t led_state) {
    if (uart_cmd_queue == NULL) {
        return ESP_FAIL;
    }
    
    uart_cmd_t cmd = {
        .type = CMD_TYPE_LED_CONTROL,
        .data[0] = led_state,
        .data_len = 1,
        .timeout_ms = UART_CMD_TIMEOUT_MS,
        .retry_count = 0
    };
    
    // Try to send with timeout - if queue is full, log warning
    if (xQueueSendToFront(uart_cmd_queue, &cmd, pdMS_TO_TICKS(50)) == pdTRUE) {
        ESP_LOGI(TAG, "[LED CMD] Enqueued LED command: %s", led_state ? "ON" : "OFF");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "[LED CMD] Failed to enqueue (queue full or busy)");
        return ESP_FAIL;
    }
}

// Get latest sensor data (thread-safe read)
sensor_data_t get_sensor_data(void) {
    // Check if data is stale (no update for 6 seconds)
    int64_t now = esp_timer_get_time();
    if (g_sensor_data.valid && (now - g_sensor_data.timestamp) > 6000000) {
        ESP_LOGW(TAG, "[STALE] Sensor data older than 6 seconds");
        g_sensor_data.valid = false;
    }
    
    return g_sensor_data;
}

// Clear RX buffer (reset frame synchronization)
static void clear_rx_buffer(void) {
    uart_flush_input(UART_NUM);
    uart_rx_index = 0;
    memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
    ESP_LOGD(TAG, "RX buffer cleared (frame resync)");
}

// Read and accumulate bytes from UART (non-blocking)
static int read_available_bytes(uint8_t *buffer, int max_len, int timeout_ms) {
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    int total_read = 0;
    
    while (total_read < max_len && timeout_ticks > 0) {
        int len = uart_read_bytes(UART_NUM, &buffer[total_read], 
                                   max_len - total_read, timeout_ticks);
        
        if (len > 0) {
            total_read += len;
            timeout_ticks = pdMS_TO_TICKS(100);  // Reduce timeout for next batch
        } else if (len == 0) {
            break;  // Timeout
        } else {
            ESP_LOGE(TAG, "UART read error: %d", len);
            break;
        }
    }
    
    return total_read;
}

// Parse frame from STM32: [0xAA][TYPE][LEN][DATA...][0x55]
// No checksum needed
static bool parse_frame(uint8_t *buf, int len, sensor_data_t *data) {
    if (len < 4) {  // MIN: [0xAA][TYPE][LEN][0x55]
        ESP_LOGW(TAG, "[PARSE] Buffer too short: %d bytes", len);
        return false;
    }
    
    // Find START byte
    int start_idx = -1;
    for (int i = 0; i < len; i++) {
        if (buf[i] == FRAME_START) {
            start_idx = i;
            break;
        }
    }
    
    if (start_idx == -1) {
        ESP_LOGW(TAG, "[PARSE] No START byte (0xAA) found");
        return false;
    }
    
    uint8_t *frame = &buf[start_idx];
    int available = len - start_idx;
    
    if (available < 4) {
        ESP_LOGW(TAG, "[PARSE] Incomplete after START: %d bytes", available);
        return false;
    }
    
    uint8_t data_len = frame[2];
    
    // Validate length
    if (data_len > 32) {
        ESP_LOGW(TAG, "[PARSE] Invalid length: %d", data_len);
        return false;
    }
    
    // Need: START(1) + TYPE(1) + LEN(1) + DATA(data_len) + END(1)
    int required = 1 + 1 + 1 + data_len + 1;
    if (available < required) {
        ESP_LOGW(TAG, "[PARSE] Incomplete frame: need %d, have %d", required, available);
        return false;
    }
    
    // Check END byte
    if (frame[3 + data_len] != FRAME_END) {
        ESP_LOGW(TAG, "[PARSE] END byte (0x55) not found at offset %d", 3 + data_len);
        return false;
    }
    
    // Parse sensor data: [TEMP][HUM][LED]
    if (data_len >= 3) {
        uint8_t temp = frame[3];
        uint8_t hum = frame[4];
        uint8_t led = frame[5];
        
        // Sanity check (temp: 0-80°C, hum: 0-100%, led: 0-1)
        if (temp <= 80 && hum <= 100 && led <= 1) {
            data->temperature = (float)temp;
            data->humidity = (float)hum;
            data->led_state = led;
            data->timestamp = esp_timer_get_time();
            data->valid = true;
            
            ESP_LOGI(TAG, "[RX VALID] %.1f°C, %.1f%% RH, LED=%d", 
                     data->temperature, data->humidity, data->led_state);
            return true;
        } else {
            ESP_LOGW(TAG, "[PARSE] Data out of range: T=%d, H=%d, LED=%d", 
                     temp, hum, led);
            return false;
        }
    }
    
    ESP_LOGW(TAG, "[PARSE] Data length too short: %d", data_len);
    return false;
}

// Send command to STM32
// Frame format: [0xAA][TYPE][LEN][DATA...][0x55]
// Returns: 0 = success, -1 = UART error
static int send_command_frame(uart_cmd_t *cmd) {
    uint8_t frame[16];
    int frame_len = 0;
    
    frame[frame_len++] = FRAME_START;  // 0xAA
    frame[frame_len++] = cmd->type;     // Command type
    frame[frame_len++] = cmd->data_len; // Data length
    
    // Add data if any
    for (int i = 0; i < cmd->data_len; i++) {
        frame[frame_len++] = cmd->data[i];
    }
    
    frame[frame_len++] = FRAME_END;    // 0x55
    
    ESP_LOGI(TAG, "[TX] Command 0x%02X (frame_len=%d) [should be 4 for sensor_request]: ", cmd->type, frame_len);
    ESP_LOG_BUFFER_HEX(TAG, frame, frame_len);
    
    int written = uart_write_bytes(UART_NUM, (const char*)frame, frame_len);
    if (written == frame_len) {
        return 0;
    } else {
        ESP_LOGE(TAG, "[TX] Write failed: wrote %d of %d bytes", written, frame_len);
        return -1;
    }
}

// Execute single command with retry logic
static uart_cmd_result_t execute_command(uart_cmd_t *cmd) {
    uint8_t attempt = 0;
    
    while (attempt <= cmd->retry_count) {
        // Clear buffer before command (flush any stale data from UART HW)
        uart_flush_input(UART_NUM);
        uart_rx_index = 0;
        
        // Send command to STM32
        if (send_command_frame(cmd) != 0) {
            ESP_LOGE(TAG, "[EXEC] Attempt %d: TX failed", attempt + 1);
            attempt++;
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        
        // Wait for response
        uint8_t rx_buffer[128];
        int rx_len = read_available_bytes(rx_buffer, sizeof(rx_buffer), 
                                         (int)cmd->timeout_ms);
        
        if (rx_len <= 0) {
            ESP_LOGW(TAG, "[EXEC] Attempt %d: Timeout (no response)", attempt + 1);
            attempt++;
            
            if (attempt <= cmd->retry_count) {
                vTaskDelay(pdMS_TO_TICKS(UART_INTER_CMD_DELAY));
            }
            continue;
        }
        
        // Parse response
        sensor_data_t temp_data;
        ESP_LOGD(TAG, "[RX] Raw response (%d bytes):", rx_len);
        ESP_LOG_BUFFER_HEX(TAG, rx_buffer, rx_len > 32 ? 32 : rx_len);
        
        if (parse_frame(rx_buffer, rx_len, &temp_data)) {
            // Update global sensor data
            g_sensor_data = temp_data;
            
            if (cmd->type == CMD_TYPE_SENSOR_REQUEST) {
                ESP_LOGI(TAG, "[EXEC] Sensor data updated successfully");
            } else if (cmd->type == CMD_TYPE_LED_CONTROL) {
                ESP_LOGI(TAG, "[EXEC] LED command confirmed: LED=%d", temp_data.led_state);
            }
            
            return UART_CMD_SUCCESS;
        } else {
            ESP_LOGW(TAG, "[EXEC] Attempt %d: Parse failed (rx_len=%d)", attempt + 1, rx_len);
            attempt++;
            
            if (attempt <= cmd->retry_count) {
                vTaskDelay(pdMS_TO_TICKS(UART_INTER_CMD_DELAY));
            }
            continue;
        }
    }
    
    // All retries exhausted
    if (cmd->retry_count > 0) {
        ESP_LOGE(TAG, "[EXEC] Command failed after %d attempts (max retries exceeded)", 
                 attempt);
        return UART_CMD_RETRY_EXCEEDED;
    } else {
        return UART_CMD_TIMEOUT;
    }
}

// Main UART command executor task
void uart_executor_task(void *arg) {
    ESP_LOGI(TAG, "[EXECUTOR] Started (waiting for commands)");
    
    if (uart_cmd_queue == NULL) {
        ESP_LOGE(TAG, "[EXECUTOR] Queue not initialized");
        vTaskDelete(NULL);
        return;
    }
    
    uart_cmd_t cmd;
    
    while (1) {
        // Wait for command in queue
        if (xQueueReceive(uart_cmd_queue, &cmd, pdMS_TO_TICKS(500)) == pdTRUE) {
            // Check gateway connection
            if (!g_connected) {
                ESP_LOGW(TAG, "[EXECUTOR] Gateway not connected, skipping command");
                continue;
            }
            
            // Execute command
            uart_cmd_result_t result = execute_command(&cmd);
            
            if (result != UART_CMD_SUCCESS) {
                ESP_LOGW(TAG, "[EXECUTOR] Command failed: result=%d", result);
                // Mark sensor data as invalid on failure
                g_sensor_data.valid = false;
            }
            
            // Inter-command delay
            vTaskDelay(pdMS_TO_TICKS(UART_INTER_CMD_DELAY));
        } else {
            // Queue empty or timeout - this is normal
            if (g_connected) {
                // If gateway is connected but no sensor requests, enqueue periodic request
                // This ensures we get fresh sensor data even if no explicit request
                uart_enqueue_sensor_request();
            }
        }
    }
}

