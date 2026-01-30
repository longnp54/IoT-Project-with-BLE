/*
 * UART Handler - Request-Response with STM32
 * ESP32 requests, STM32 responds with sensor data
 */

#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// UART Configuration
#define UART_NUM           UART_NUM_1
#define UART_TX_PIN        17  // TX to STM32 RX (for requests)
#define UART_RX_PIN        16  // RX from STM32 TX (for responses)
#define UART_BAUD          115200
#define RX_BUF_SIZE        1024

// Protocol Request-Response với STM32
// ESP32 → STM32 (Request): [0xAA][CMD][CHECKSUM][0x55]
// STM32 → ESP32 (Response): [0xAA][LEN][DATA...][CHECKSUM][0x55]
#define FRAME_START        0xAA
#define FRAME_END          0x55
#define CMD_REQUEST_DATA   0x01  // Lệnh yêu cầu sensor data
#define CMD_LED_CONTROL    0x02  // Lệnh điều khiển LED

// Notification bit for LED command (bits 0-7 reserved for LED state)
#define LED_CMD_NOTIFICATION_BIT  (1UL << 31)

// Sensor data structure
typedef struct {
    float temperature;      // Nhiệt độ (°C)
    float humidity;         // Độ ẩm (%)
    uint8_t led_state;      // Trạng thái LED: 0=OFF, 1=ON
    uint64_t timestamp;     // Thời gian nhận (microseconds)
    bool valid;             // Data có hợp lệ không
} sensor_data_t;

// Function prototypes
esp_err_t uart_init(void);
esp_err_t uart_request_sensor_data(void);  // Gửi request tới STM32
esp_err_t uart_send_led_command(uint8_t led_state);  // Gửi lệnh LED
void uart_receive_task(void *arg);
sensor_data_t get_sensor_data(void);

// UART task handle - exported for LED command notification
extern TaskHandle_t uart_task_handle;

#endif // UART_HANDLER_H
