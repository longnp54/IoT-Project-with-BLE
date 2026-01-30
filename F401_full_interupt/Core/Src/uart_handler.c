#include "uart_handler.h"
#include <string.h>
#include <stdio.h>

// ============ GLOBAL VARIABLES ============
SensorData_t g_sensor_data = {25, 50, 0};

// UART instance
static UART_HandleTypeDef *huart_instance;

// RX frame buffer (simple circular)
#define RX_BUFFER_SIZE 256
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_write_idx = 0;
static uint16_t rx_read_idx = 0;

// Frame parsing state
#define FRAME_MAX_LEN 64
static uint8_t frame[FRAME_MAX_LEN];
static uint16_t frame_len = 0;
static int frame_complete = 0;

// Single byte RX for interrupt
static uint8_t rx_byte;

// External DHT11
extern DHT11_Data_t dht_data;

/**
 * @brief Initialize UART Handler
 */
void UART_Handler_Init(UART_HandleTypeDef *huart) {
    huart_instance = huart;
    rx_write_idx = 0;
    rx_read_idx = 0;
    frame_len = 0;
    frame_complete = 0;
    
    // Start interrupt RX for 1 byte at a time
    HAL_UART_Receive_IT(huart_instance, &rx_byte, 1);
}

/**
 * @brief UART RX Callback - Called for each received byte
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) return;
    
    // Add byte to circular buffer
    rx_buffer[rx_write_idx] = rx_byte;
    rx_write_idx = (rx_write_idx + 1) % RX_BUFFER_SIZE;
    
    // Re-enable interrupt for next byte
    HAL_UART_Receive_IT(huart_instance, &rx_byte, 1);
}

/**
 * @brief Process buffered RX data (call from main loop)
 */
void UART_Process(void) {
    // If frame already being processed, skip
    if (frame_complete) {
        return;
    }
    
    // Process bytes from buffer
    while (rx_read_idx != rx_write_idx) {
        uint8_t byte = rx_buffer[rx_read_idx];
        rx_read_idx = (rx_read_idx + 1) % RX_BUFFER_SIZE;
        
        // Look for START byte
        if (frame_len == 0) {
            if (byte == FRAME_START_BYTE) {
                frame[0] = byte;
                frame_len = 1;
            }
            continue;
        }
        
        // Receiving frame
        if (frame_len < FRAME_MAX_LEN) {
            frame[frame_len] = byte;
            frame_len++;
            
            // Check if we have [AA][TYPE][LEN] to know expected length
            if (frame_len == 3) {
                uint8_t len = frame[2];
                uint16_t expected_total = 1 + 1 + 1 + len + 1;  // START+TYPE+LEN+DATA+END
                
                if (expected_total > FRAME_MAX_LEN || expected_total < 4) {
                    frame_len = 0;
                    continue;
                }
            }
            
            // Check if END byte received at expected position
            if (frame_len >= 4) {
                uint8_t len = frame[2];
                uint16_t expected_total = 1 + 1 + 1 + len + 1;
                
                if (frame_len == expected_total) {
                    if (frame[frame_len - 1] == FRAME_END_BYTE) {
                        frame_complete = 1;  // Signal to process
                        return;  // Exit to let Process_Frame handle it
                    } else {
                        frame_len = 0;
                    }
                }
            }
        }
    }
}

/**
 * @brief Process received frame (call from main after UART_Process)
 */
void UART_Process_Frame(void) {
    if (!frame_complete || frame_len == 0) return;
    
    frame_complete = 0;  // Reset flag
    
    // Parse frame
    uint8_t type = frame[1];
    uint8_t len = frame[2];
    uint8_t *data = &frame[3];
    
    // Handle SENSOR REQUEST
    if (type == CMD_SENSOR_REQUEST && len == 0) {
        Read_DHT11_Sensor();
    }
    // Handle LED CONTROL
    else if (type == CMD_LED_CONTROL && len == 1) {
        uint8_t led_cmd = data[0];
        
        if (led_cmd == 0x00) {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            g_sensor_data.led_state = 0;
        } else if (led_cmd == 0x01) {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            g_sensor_data.led_state = 1;
        }
    }
    
    // Send response: [AA][01][03][TEMP][HUM][LED][55]
    uint8_t response[7];
    response[0] = FRAME_START_BYTE;          // 0xAA
    response[1] = 0x01;                      // TYPE
    response[2] = 0x03;                      // LEN
    response[3] = g_sensor_data.temperature; // TEMP
    response[4] = g_sensor_data.humidity;    // HUM
    response[5] = g_sensor_data.led_state;   // LED
    response[6] = FRAME_END_BYTE;            // 0x55
    
    HAL_UART_Transmit(huart_instance, response, 7, 100);
    
    // Reset for next frame
    frame_len = 0;
}

/**
 * @brief Read DHT11 and update global sensor data
 */
void Read_DHT11_Sensor(void) {
    if (DHT11_Read(&dht_data)) {
        if (dht_data.valid) {
            g_sensor_data.temperature = dht_data.temperature;
            g_sensor_data.humidity = dht_data.humidity;
            
            // Clamp values
            if (g_sensor_data.temperature > 50) g_sensor_data.temperature = 50;
            if (g_sensor_data.humidity > 100) g_sensor_data.humidity = 100;
        } else {
            g_sensor_data.temperature = 25;
            g_sensor_data.humidity = 50;
        }
    } else {
        g_sensor_data.temperature = 25;
        g_sensor_data.humidity = 50;
    }
}
