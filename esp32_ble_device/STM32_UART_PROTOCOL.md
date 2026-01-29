# 📡 STM32 UART Protocol - Hướng Dẫn Cho STM32 Developer

## 🎯 Mục Đích
STM32 nhận **request** từ ESP32, đọc cảm biến DHT11, và gửi **response** về cho ESP32.

---

## ⚙️ Cấu Hình UART

```c
// UART Configuration
Baud Rate: 115200
Data Bits: 8
Parity: None
Stop Bits: 1
Flow Control: None

// Pins (ví dụ - tùy board STM32)
TX Pin: PA9  (kết nối đến ESP32 RX GPIO16)
RX Pin: PA10 (kết nối từ ESP32 TX GPIO17)
```

---

## 📥 Frame REQUEST (ESP32 → STM32)

**Kích thước:** 4 bytes

```
┌──────┬──────┬──────────┬──────┐
│ 0xAA │ 0x01 │ CHECKSUM │ 0x55 │
└──────┴──────┴──────────┴──────┘
  START  CMD    XOR(CMD)   END
```

### Chi tiết:
- **Byte 0:** `0xAA` - START byte
- **Byte 1:** `0x01` - CMD_REQUEST_DATA (lệnh yêu cầu sensor data)
- **Byte 2:** `CHECKSUM` - XOR của byte 1 (= `0x01`)
- **Byte 3:** `0x55` - END byte

### Ví dụ thực tế:
```
Hex: AA 01 01 55
     │   │  │  └─ END
     │   │  └──── CHECKSUM (0x01 XOR = 0x01)
     │   └─────── CMD
     └─────────── START
```

---

## 📤 Frame RESPONSE (STM32 → ESP32)

**Kích thước:** 7 bytes (DHT11 - độ phân giải 8-bit)

```
┌──────┬─────┬──────┬─────┬─────┬──────────┬──────┐
│ 0xAA │ LEN │ TEMP │ HUM │ LED │ CHECKSUM │ 0x55 │
└──────┴─────┴──────┴─────┴─────┴──────────┴──────┘
 START  0x03   °C     %    0/1   XOR(...)   END
```

### Chi tiết:
- **Byte 0:** `0xAA` - START byte
- **Byte 1:** `0x03` - Length (3 bytes data)
- **Byte 2:** `TEMP` - Nhiệt độ (0-50°C) - **giá trị nguyên**
- **Byte 3:** `HUM` - Độ ẩm (20-90%RH) - **giá trị nguyên**
- **Byte 4:** `LED` - Trạng thái LED (0=OFF, 1=ON)
- **Byte 5:** `CHECKSUM` - XOR của [LEN + TEMP + HUM + LED]
- **Byte 6:** `0x55` - END byte

### Ví dụ thực tế:
```
Nhiệt độ: 25°C, Độ ẩm: 65%, LED: ON

Hex: AA 03 19 41 01 [CS] 55
     │  │  │  │  │   │   └─ END
     │  │  │  │  │   └────── CHECKSUM
     │  │  │  │  └────────── LED = 1 (ON)
     │  │  │  └───────────── HUM = 0x41 = 65
     │  │  └──────────────── TEMP = 0x19 = 25
     │  └─────────────────── LEN = 3
     └────────────────────── START

CHECKSUM = 0x03 XOR 0x19 XOR 0x41 XOR 0x01 = 0x58
Kết quả: AA 03 19 41 01 58 55
```

---

## 🔢 Tính Checksum (XOR)

```c
uint8_t calc_checksum(uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum ^= data[i];
    }
    return sum;
}

// Sử dụng:
// checksum = calc_checksum(&frame[1], 4);  // XOR của [LEN, TEMP, HUM, LED]
```

---

## 💻 Code Mẫu Cho STM32

### 1. Parse Request từ ESP32

```c
#define FRAME_START 0xAA
#define FRAME_END   0x55
#define CMD_REQUEST 0x01

typedef struct {
    uint8_t start;
    uint8_t cmd;
    uint8_t checksum;
    uint8_t end;
} request_frame_t;

bool parse_request(uint8_t *rx_buf, uint8_t len) {
    if (len < 4) return false;
    
    // Kiểm tra frame
    if (rx_buf[0] != FRAME_START || rx_buf[3] != FRAME_END) {
        return false;
    }
    
    // Kiểm tra checksum
    uint8_t calc_cs = rx_buf[1]; // XOR của CMD
    if (rx_buf[2] != calc_cs) {
        return false;
    }
    
    // Kiểm tra CMD
    if (rx_buf[1] == CMD_REQUEST) {
        return true;
    }
    
    return false;
}
```

### 2. Đọc DHT11

```c
// Giả sử bạn đã có hàm đọc DHT11
extern uint8_t DHT11_Read_Temperature(void);  // Trả về 0-50
extern uint8_t DHT11_Read_Humidity(void);     // Trả về 20-90

// Hoặc struct
typedef struct {
    uint8_t temperature;  // °C
    uint8_t humidity;     // %RH
} dht11_data_t;

extern dht11_data_t DHT11_Read(void);
```

### 3. Gửi Response về ESP32

```c
void send_sensor_response(uint8_t temp, uint8_t hum, uint8_t led_state) {
    uint8_t response[7];
    
    // Build frame
    response[0] = FRAME_START;          // 0xAA
    response[1] = 0x03;                 // LEN = 3 bytes
    response[2] = temp;                 // Temperature
    response[3] = hum;                  // Humidity
    response[4] = led_state;            // LED state
    
    // Calculate checksum (LEN + TEMP + HUM + LED)
    response[5] = response[1] ^ response[2] ^ response[3] ^ response[4];
    
    response[6] = FRAME_END;            // 0x55
    
    // Send via UART
    HAL_UART_Transmit(&huart1, response, 7, 100);
}
```

### 4. Main Loop

```c
void UART_Task(void) {
    uint8_t rx_buffer[10];
    uint8_t rx_len = 0;
    
    // Đọc UART (non-blocking hoặc interrupt)
    if (HAL_UART_Receive(&huart1, rx_buffer, 4, 100) == HAL_OK) {
        // Parse request
        if (parse_request(rx_buffer, 4)) {
            // Đọc cảm biến DHT11
            uint8_t temp = DHT11_Read_Temperature();
            uint8_t hum = DHT11_Read_Humidity();
            uint8_t led = HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin);
            
            // Gửi response
            send_sensor_response(temp, hum, led);
        }
    }
}
```

---

## 🔄 Flowchart

```
┌─────────────────────────────────────┐
│   STM32 UART Interrupt Handler      │
└─────────────────────────────────────┘
                ↓
┌─────────────────────────────────────┐
│  Nhận 4 bytes từ ESP32              │
│  [0xAA][0x01][0x01][0x55]           │
└─────────────────────────────────────┘
                ↓
        ┌───────────────┐
        │ Parse Request │
        │ Valid?        │
        └───────────────┘
         Yes ↓       ↓ No
             ↓       └──→ Bỏ qua
┌─────────────────────────────────────┐
│  Đọc DHT11                          │
│  • Temperature: 25°C                │
│  • Humidity: 65%                    │
│  • LED: GPIO Read                   │
└─────────────────────────────────────┘
                ↓
┌─────────────────────────────────────┐
│  Build Response Frame               │
│  [0xAA][0x03][25][65][1][CS][0x55] │
└─────────────────────────────────────┘
                ↓
┌─────────────────────────────────────┐
│  UART Transmit 7 bytes              │
└─────────────────────────────────────┘
                ↓
        ┌──────────────┐
        │ Chờ request  │
        │ tiếp theo    │
        └──────────────┘
```

---

## ⚡ Code Hoàn Chỉnh STM32 (HAL)

```c
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
#define FRAME_START 0xAA
#define FRAME_END   0x55
#define CMD_REQUEST 0x01
/* USER CODE END PV */

/* USER CODE BEGIN 0 */

// Tính checksum XOR
uint8_t calc_checksum(uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum ^= data[i];
    }
    return sum;
}

// Parse request từ ESP32
bool parse_request(uint8_t *rx_buf, uint8_t len) {
    if (len < 4) return false;
    if (rx_buf[0] != FRAME_START || rx_buf[3] != FRAME_END) return false;
    if (rx_buf[2] != rx_buf[1]) return false;  // Checksum
    if (rx_buf[1] == CMD_REQUEST) return true;
    return false;
}

// Gửi response về ESP32
void send_sensor_response(UART_HandleTypeDef *huart, uint8_t temp, uint8_t hum, uint8_t led) {
    uint8_t response[7];
    
    response[0] = FRAME_START;
    response[1] = 0x03;  // LEN
    response[2] = temp;
    response[3] = hum;
    response[4] = led;
    response[5] = calc_checksum(&response[1], 4);  // Checksum
    response[6] = FRAME_END;
    
    HAL_UART_Transmit(huart, response, 7, 100);
}

/* USER CODE END 0 */

int main(void) {
    /* USER CODE BEGIN 1 */
    uint8_t rx_buffer[10];
    /* USER CODE END 1 */
    
    /* Initialize all configured peripherals */
    MX_UART1_Init();
    MX_DHT11_Init();
    
    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        // Nhận request từ ESP32
        if (HAL_UART_Receive(&huart1, rx_buffer, 4, 200) == HAL_OK) {
            
            // Kiểm tra request hợp lệ
            if (parse_request(rx_buffer, 4)) {
                
                // Đọc DHT11
                uint8_t temp = DHT11_Read_Temperature();  // 0-50
                uint8_t hum = DHT11_Read_Humidity();      // 20-90
                
                // Đọc LED state
                uint8_t led = HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin);
                
                // Gửi response
                send_sensor_response(&huart1, temp, hum, led);
            }
        }
        
        HAL_Delay(10);  // Small delay
        
    /* USER CODE END WHILE */
    }
}
```

---

## 🧪 Test Cases

### Test 1: Request hợp lệ
**Input:** `AA 01 01 55`  
**Expected Output:** `AA 03 [TEMP] [HUM] [LED] [CS] 55`

### Test 2: Nhiệt độ 25°C, Độ ẩm 65%, LED ON
**Response:** `AA 03 19 41 01 58 55`

### Test 3: Nhiệt độ 30°C, Độ ẩm 80%, LED OFF
**Response:** `AA 03 1E 50 00 6D 55`

### Test 4: Request sai START byte
**Input:** `BB 01 01 55`  
**Expected:** Không gửi response

### Test 5: Request sai checksum
**Input:** `AA 01 FF 55`  
**Expected:** Không gửi response

---

## 🔧 Debug Tips

### 1. Kiểm tra UART
```c
// Test UART bằng echo
uint8_t test[] = "STM32 UART OK\r\n";
HAL_UART_Transmit(&huart1, test, sizeof(test), 100);
```

### 2. Log frame nhận được
```c
printf("RX: %02X %02X %02X %02X\r\n", 
       rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
```

### 3. Log frame gửi đi
```c
printf("TX: %02X %02X %02X %02X %02X %02X %02X\r\n",
       response[0], response[1], response[2], response[3], 
       response[4], response[5], response[6]);
```

---

## 📌 Lưu Ý

1. **DHT11 cần delay 2s giữa các lần đọc** - ESP32 đã xử lý (request mỗi 2s)
2. **Nhiệt độ và độ ẩm là giá trị nguyên** (uint8_t) - không có số thập phân
3. **Timeout UART:** Nên set ~200ms để đủ thời gian xử lý
4. **Checksum quan trọng:** ESP32 sẽ reject frame nếu checksum sai
5. **Byte order:** Big-endian (MSB first) - không áp dụng vì chỉ dùng 1 byte

---

## 🎯 Checklist

- [ ] UART config: 115200, 8N1
- [ ] Pins kết nối đúng: TX→RX, RX→TX, GND chung
- [ ] Parse request đúng format
- [ ] Đọc DHT11 thành công
- [ ] Tính checksum đúng
- [ ] Gửi response đủ 7 bytes
- [ ] Test với ESP32 thực tế

---

## 📞 Support

Nếu có vấn đề, kiểm tra:
1. Kết nối UART (TX/RX có đúng không?)
2. Baud rate (115200)
3. Checksum calculation
4. DHT11 library hoạt động

Good luck! 🚀
