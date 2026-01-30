#ifndef DHT11_H
#define DHT11_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// DHT11 GPIO Configuration
#define DHT11_PORT      GPIOA
#define DHT11_PIN       GPIO_PIN_1

// DHT11 Data structure
typedef struct {
    uint8_t temperature;
    uint8_t humidity;
    bool valid;
} DHT11_Data_t;

// Function prototypes
void DHT11_Init(void);
bool DHT11_Read(DHT11_Data_t *data);

#endif
