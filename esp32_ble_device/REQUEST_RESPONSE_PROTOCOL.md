# 🔄 Request-Response UART Protocol

## 📡 Giao thức mới (ESP32 chủ động)

### ESP32 → STM32 (Request):
```
[0xAA] [0x01] [CHECKSUM] [0x55]
 START   CMD    XOR(CMD)   END

Total: 4 bytes
```

### STM32 → ESP32 (Response):
```
[0xAA] [0x05] [TEMP_H] [TEMP_L] [HUM_H] [HUM_L] [LED] [CHECKSUM] [0x55]
 START  LEN     Temperature   Humidity    State   XOR(...)   END

Total: 9 bytes
```

---

## ⏱️ Timing Diagram

```
ESP32                          STM32
  │                              │
  ├──── Request ────────────────>│
  │   [0xAA 0x01 0x01 0x55]      │
  │                              │
  │                              ├─ Đọc DHT11
  │                              │  (200ms)
  │                              │
  │<──── Response ───────────────┤
  │   [0xAA 0x05 ... 0x55]       │
  │                              │
  ├─ Parse & Update data         │
  │                              │
  ├─ Wait 2 seconds              │
  │                              │
  ├──── Request ────────────────>│
  │                              │
  ...                           ...
```

**Chu kỳ:** 2 giây/lần request

---

## 🎯 Ưu điểm

✅ ESP32 kiểm soát hoàn toàn timing  
✅ STM32 không spam data  
✅ Đồng bộ hóa tốt hơn  
✅ Dễ debug (biết rõ request/response)  
✅ Tiết kiệm năng lượng STM32  

---

## 🔧 Flow ESP32

```c
uart_receive_task() {
    while(1) {
        1. uart_request_sensor_data()    // Gửi request
        2. uart_read_bytes(timeout=200ms) // Đợi response
        3. parse_frame()                  // Parse data
        4. update g_sensor_data           // Lưu
        5. vTaskDelay(2000ms)             // Chờ 2s
    }
}
```

---

## 🔧 Flow STM32

```c
HAL_UART_RxCpltCallback() {
    if (received_request_valid) {
        1. DHT11_Read(&temp, &hum)
        2. Get LED state
        3. send_sensor_response(temp, hum, led)
    }
}
```

---

## 📋 Checksum Calculation

```c
// ESP32 Request checksum:
checksum = 0x01  // Only CMD byte

// STM32 Response checksum:
checksum = LEN ^ TEMP_H ^ TEMP_L ^ HUM_H ^ HUM_L ^ LED
         = 0x05 ^ ... ^ ...
```

---

## 🧪 Test Example

### ESP32 gửi:
```
0xAA 0x01 0x01 0x55
```

### STM32 trả về (25.5°C, 60.3%, LED ON):
```
0xAA 0x05 0x09 0xF6 0x17 0x8E 0x01 [CS] 0x55
      │    └─────┬─────┘ └─────┬─────┘ │
      │      2550 (25.5°C)  6030 (60.3%) LED=1
      Length=5
```

---

## 🐛 Error Handling

### ESP32:
- Timeout 200ms → `g_sensor_data.valid = false`
- Invalid checksum → Bỏ qua, thử lại lần sau
- No START/END → Bỏ qua frame

### STM32:
- Invalid request → Không response
- DHT11 lỗi → Gửi data cũ hoặc 0xFF
- UART busy → Buffer request

---

**Code đã update xong!** 🚀
