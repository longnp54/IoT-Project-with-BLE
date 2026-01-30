/*
 * UART Handler - Queue-based Command Pipeline
 * Manages command queue with proper synchronization and ACK/NACK handling
 */

#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// UART Configuration
#define UART_NUM           UART_NUM_1
#define UART_TX_PIN        17  // TX to STM32 RX (for requests)
#define UART_RX_PIN        16  // RX from STM32 TX (for responses)
#define UART_BAUD          115200
#define RX_BUF_SIZE        1024

// Protocol Request-Response with STM32 (NO CHECKSUM)
// ESP32 → STM32 (Request): [0xAA][TYPE][LEN][DATA...][0x55]
// STM32 → ESP32 (Response): [0xAA][LEN][DATA...][0x55]
#define FRAME_START        0xAA
#define FRAME_END          0x55
#define CMD_REQUEST_DATA   0x01  // Request sensor data
#define CMD_LED_CONTROL    0x02  // LED control command
#define CMD_ACK            0x7F  // ACK response from STM32
#define CMD_NACK           0x7E  // NACK response from STM32

// Command timeout and retry
#define UART_CMD_TIMEOUT_MS    1500  // Response timeout per command
#define UART_CMD_MAX_RETRIES   2     // Max retries per command
#define UART_INTER_CMD_DELAY   100   // Delay between commands (ms)

// Command queue maximum
#define UART_CMD_QUEUE_SIZE    10

// Command types
typedef enum {
    CMD_TYPE_SENSOR_REQUEST = 0x01,
    CMD_TYPE_LED_CONTROL = 0x02,
} uart_cmd_type_t;

// Command structure for queue
typedef struct {
    uart_cmd_type_t type;
    uint8_t data[4];           // Additional data (e.g., LED state)
    uint8_t data_len;
    uint32_t timeout_ms;
    uint8_t retry_count;
} uart_cmd_t;

// Sensor data structure
typedef struct {
    float temperature;         // Temperature (°C)
    float humidity;            // Humidity (%)
    uint8_t led_state;         // LED state: 0=OFF, 1=ON
    uint64_t timestamp;        // Reception time (microseconds)
    bool valid;                // Data validity flag
} sensor_data_t;

// Command execution result
typedef enum {
    UART_CMD_SUCCESS = 0,
    UART_CMD_TIMEOUT = 1,
    UART_CMD_INVALID_RESPONSE = 2,
    UART_CMD_RETRY_EXCEEDED = 3,
} uart_cmd_result_t;

// Function prototypes
esp_err_t uart_init(void);

// Enqueue commands
esp_err_t uart_enqueue_sensor_request(void);
esp_err_t uart_enqueue_led_command(uint8_t led_state);

// Get sensor data
sensor_data_t get_sensor_data(void);

// Task functions
void uart_executor_task(void *arg);    // Main UART command executor

// Command queue handle - exported
extern QueueHandle_t uart_cmd_queue;

#endif // UART_HANDLER_H
