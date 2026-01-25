# 🚀 Quick Start Guide - ESP32 Environmental Sensor Device

## ✅ Code đã hoàn chỉnh!

### 📁 Files trong project:
```
main/
├── gatts_demo.c       ✅ BLE GATT Server + Main (ĐÃ SỬA)
├── uart_handler.c     ✅ UART RX handler (MỚI)
├── uart_handler.h     ✅ UART header (MỚI)
└── CMakeLists.txt     ✅ Đã có uart_handler.c

STM32_TEST_CODE.c      ✅ Code mẫu cho STM32
```

---

## 🔥 Cách hoạt động:

```
STM32 (2s/lần)  →  UART  →  ESP32  →  BLE  →  Gateway
 DHT11 data         115200   Parse      Notify   Nhận data
```

### 📊 Luồng xử lý trong ESP32:

1. **UART Task** (`uart_receive_task`): 
   - Đọc liên tục từ UART
   - Parse frame từ STM32
   - Update global `g_sensor_data`

2. **BLE Notify Task** (`ble_notify_task`):
   - Chạy mỗi 2s
   - Kiểm tra `notify_enabled`
   - Gửi sensor data qua BLE notification

3. **BLE Event Handler**:
   - READ: Trả về sensor data hiện tại
   - WRITE Descriptor: Enable/Disable notification
   - DISCONNECT: Reset notification state

---

## 🛠️ Build & Flash

### Bước 1: Build
```bash
cd d:\esp-idf\esp32_ble\esp32_ble_device
idf.py build
```

### Bước 2: Flash
```bash
idf.py -p COM3 flash
```

### Bước 3: Monitor
```bash
idf.py -p COM3 monitor
```

---

## 📱 Test với nRF Connect (trước khi có STM32)

### 1. Scan & Connect:
- Mở nRF Connect app
- Scan → tìm `ENV_SENSOR`
- Connect

### 2. Test READ:
- Vào Service `0x00FF`
- Characteristic `0xFF01` → nhấn READ
- Sẽ thấy 6 bytes: `[00 00 00 00 00 00]` (chưa có data từ STM32)

### 3. Enable Notification:
- Nhấn icon 3 mũi tên xuống
- Sẽ nhận notification mỗi 2s (data fake ban đầu)

---

## 🔌 Kết nối STM32

### Hardware:
```
STM32           ESP32
──────         ──────
TX   ────────> GPIO16 (RX)
GND  ────────> GND

(GPIO17 TX của ESP32 không dùng, để dành mở rộng)
```

### STM32 Code:
Copy file `STM32_TEST_CODE.c` vào project STM32, gọi:

```c
// Trong main loop:
while(1) {
    DHT11_Read(&temp, &hum);
    send_sensor_data(temp, hum, led_state);
    HAL_Delay(2000);
}
```

---

## 📝 Log Monitor - Khi hoạt động tốt:

```
I (1234) ESP32_DEVICE: Advertising start successfully
I (1250) UART_HANDLER: UART initialized: RX=GPIO16, Baud=115200
I (1260) UART_HANDLER: UART receive task started
I (1270) ESP32_DEVICE: BLE notify task started

// STM32 gửi data
I (3456) UART_HANDLER: Received 9 bytes from UART
I (3460) UART_HANDLER: Parsed: Temp=25.50°C, Hum=60.30%, LED=ON
I (3470) UART_HANDLER: Sensor updated successfully

// Gateway connect
I (5234) ESP32_DEVICE: Connected, conn_id 0
I (5240) ESP32_DEVICE: Notification enabled by client

// Auto notify mỗi 2s
I (7240) ESP32_DEVICE: Notified: T=25.5°C H=60.3% LED=1
I (9240) ESP32_DEVICE: Notified: T=25.6°C H=60.4% LED=1
I (11240) ESP32_DEVICE: Notified: T=25.7°C H=60.5% LED=0
```

---

## 🐛 Troubleshooting

### ❌ Problem: "UART_HANDLER: Frame too short"
**Solution:** STM32 chưa gửi đúng format, kiểm tra:
- Baud rate đúng 115200
- Frame đúng 9 bytes
- Start byte = 0xAA, End byte = 0x55

### ❌ Problem: "Checksum mismatch"
**Solution:** 
- Kiểm tra hàm `calculate_checksum()` trong STM32
- Phải XOR từ byte[1] đến byte[6]

### ❌ Problem: Gateway không nhận notification
**Solution:**
- Kiểm tra client đã enable notification chưa (write 0x0001 vào descriptor)
- Xem log: phải có "Notification enabled by client"

### ❌ Problem: "Sensor data stale"
**Solution:**
- STM32 không gửi data liên tục
- Kiểm tra UART TX của STM32

---

## 🎯 Test Step-by-Step

### 1️⃣ Test ESP32 standalone (không cần STM32):
```bash
idf.py flash monitor
# Xem log khởi động OK không
# Test BLE với nRF Connect
```

### 2️⃣ Test UART với STM32:
```bash
# Kết nối STM32 → ESP32
# Chạy code STM32 test
# Monitor ESP32: phải thấy "Parsed: Temp=..."
```

### 3️⃣ Test end-to-end:
```bash
# STM32 chạy
# ESP32 chạy
# nRF Connect: Enable notification
# Phải thấy data update mỗi 2s
```

---

## 📊 Data Format Reference

### UART Frame (STM32 → ESP32):
```
Byte   Value    Description
─────────────────────────────────────
[0]    0xAA     Start byte
[1]    0x05     Data length (5 bytes)
[2]    0xXX     Temperature High byte
[3]    0xXX     Temperature Low byte
[4]    0xXX     Humidity High byte
[5]    0xXX     Humidity Low byte
[6]    0xXX     LED state (0/1)
[7]    0xXX     Checksum (XOR)
[8]    0x55     End byte
```

### BLE Notification (ESP32 → Gateway):
```
Byte   Description
────────────────────────────
[0]    Temperature High
[1]    Temperature Low
[2]    Humidity High
[3]    Humidity Low
[4]    LED state
[5]    Valid flag (0/1)
```

---

## ✨ Features

✅ UART RX từ STM32 (115200 baud)  
✅ Frame parsing với checksum validation  
✅ BLE GATT Server (Service 0x00FF)  
✅ Auto notification mỗi 2s  
✅ Support READ characteristic  
✅ Thread-safe sensor data  
✅ Reconnect handling  
✅ Stale data detection (>5s)  

---

## 🎉 Done!

**Code đã sẵn sàng build & flash!**

Next steps:
1. `idf.py build` → Compile
2. `idf.py flash` → Flash ESP32
3. Code STM32 (dùng STM32_TEST_CODE.c)
4. Test với nRF Connect

Có lỗi gì báo tôi nhé! 🚀
