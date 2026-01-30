#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "main.h"
#include "dht11.h"

// NEW Protocol constants - Pure Interrupt-Driven
#define FRAME_START_BYTE  0xAA
#define FRAME_END_BYTE    0x55

// Command types
#define CMD_SENSOR_REQUEST  0x01   // Sensor data request
#define CMD_LED_CONTROL     0x02   // LED control command

// Frame buffer management
#define FRAME_BUF_SIZE      256    // Circular buffer size
#define MAX_FRAME_LEN       64     // Maximum frame length

// Response constants
#define RESPONSE_TYPE       0x01   // Fixed response type
#define RESPONSE_DATA_LEN   0x03   // Always 3 bytes: TEMP, HUM, LED
#define RESPONSE_FRAME_SIZE 7      // [AA 01 03 TEMP HUM LED 55]

// LED control
#define LED_PIN             GPIO_PIN_5
#define LED_PORT            GPIOA

// Sensor data structure
typedef struct {
    uint8_t temperature;    // 0-50°C
    uint8_t humidity;       // 0-100%
    uint8_t led_state;      // 0 or 1
} SensorData_t;

// Global variables for interrupt-driven reception
extern uint8_t frame_buffer[FRAME_BUF_SIZE];
extern SensorData_t g_sensor_data;
extern uint8_t temp_rx_byte;

// Function prototypes
void UART_Handler_Init(UART_HandleTypeDef *huart);
void UART_Process(void);  // Call from main loop
void UART_Process_Frame(void);  // Call from main loop after UART_Process
void Read_DHT11_Sensor(void);

#endif
