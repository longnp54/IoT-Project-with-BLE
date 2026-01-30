#include "dht11.h"
#include "main.h"

/* ===== microsecond delay using DWT ===== */
static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

/* ===== GPIO direction ===== */
static void DHT11_SetOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

static void DHT11_SetInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // QUAN TRỌNG: Kéo lên
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_Init(void)
{
    /* Enable DWT */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    // GPIO clock đã được bật trong main.c
    // Chỉ cần cấu hình pin
    DHT11_SetOutput();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);

    // Delay 1 giây để DHT11 ổn định
    HAL_Delay(1000);
}

bool DHT11_Read(DHT11_Data_t *data)
{
    uint8_t buffer[5] = {0};
    uint32_t timeout;

    // Reset data
    data->valid = false;
    data->temperature = 0;
    data->humidity = 0;

    /* 1. Send START signal */
    DHT11_SetOutput();

    // Pull LOW for 18ms
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(18);  // Đúng 18ms

    // Pull HIGH for 20-40us
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(30);

    /* 2. Switch to INPUT mode */
    DHT11_SetInput();

    /* 3. Wait for DHT11 response - LOW for 80us */
    timeout = 10000; // Timeout ~10ms
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
        if (--timeout == 0) {
            data->humidity = 20;  // Default value
            data->temperature = 0;
            return false;
        }
        delay_us(1);
    }

    /* 4. Wait for HIGH for 80us */
    timeout = 10000;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) {
        if (--timeout == 0) {
            data->humidity = 20;
            data->temperature = 0;
            return false;
        }
        delay_us(1);
    }

    /* 5. Wait for LOW again (start of data) */
    timeout = 10000;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
        if (--timeout == 0) {
            data->humidity = 20;
            data->temperature = 0;
            return false;
        }
        delay_us(1);
    }

    /* 6. Read 40 bits (5 bytes) */
    for (int i = 0; i < 40; i++) {
        // Wait for LOW to finish (50us)
        timeout = 10000;
        while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) {
            if (--timeout == 0) {
                data->humidity = 20;
                data->temperature = 0;
                return false;
            }
        }

        // Wait 40us then check pin state
        delay_us(40);

        // If still HIGH -> bit 1, else bit 0
        if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
            buffer[i/8] |= (1 << (7 - (i % 8)));

            // Wait for HIGH to finish
            timeout = 10000;
            while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) {
                if (--timeout == 0) break;
                delay_us(1);
            }
        }
    }

    /* 7. Validate checksum */
    uint8_t checksum = buffer[0] + buffer[1] + buffer[2] + buffer[3];
    if (checksum != buffer[4]) {
        data->humidity = 20;
        data->temperature = 0;
        return false;
    }

    /* 8. Parse data - DHT11 returns integer values only */
    data->humidity = buffer[0];      // Integer humidity (20-90%)
    data->temperature = buffer[2];   // Integer temperature (0-50°C)
    data->valid = true;

    return true;
}
