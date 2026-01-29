# 📘 HƯỚNG DẪN CHI TIẾT DỰ ÁN ESP32 BLE GATEWAY

> **ESP32 BLE Gateway for Environmental Sensor** - Cổng kết nối BLE thu thập dữ liệu cảm biến và gửi lên ThingsBoard qua WiFi/MQTT

---

## 📋 MỤC LỤC

1. [Tổng quan dự án](#1-tổng-quan-dự-án)
2. [Kiến thức cơ bản cần thiết](#2-kiến-thức-cơ-bản-cần-thiết)
3. [Kiến trúc hệ thống](#3-kiến-trúc-hệ-thống)
4. [Cấu trúc code](#4-cấu-trúc-code)
5. [Luồng hoạt động chi tiết](#5-luồng-hoạt-động-chi-tiết)
6. [Giao thức dữ liệu](#6-giao-thức-dữ-liệu)
7. [Cấu hình và triển khai](#7-cấu-hình-và-triển-khai)
8. [Xử lý lỗi và debug](#8-xử-lý-lỗi-và-debug)
9. [Mở rộng và tùy chỉnh](#9-mở-rộng-và-tùy-chỉnh)

---

## 1. TỔNG QUAN DỰ ÁN

### 🎯 Mục tiêu
Xây dựng một **IoT Gateway** sử dụng ESP32 để:
- Thu thập dữ liệu từ thiết bị cảm biến môi trường qua **BLE (Bluetooth Low Energy)**
- Chuyển tiếp dữ liệu lên cloud platform **ThingsBoard** qua **WiFi/MQTT**
- Hỗ trợ điều khiển thiết bị từ xa qua **RPC (Remote Procedure Call)**

### 🔧 Phần cứng
- **ESP32**: Chip chính với WiFi + BLE dual-mode
- **ENV_SENSOR** (STM32 based): Cảm biến nhiệt độ, độ ẩm qua BLE
- **LED GPIO2**: Hiển thị trạng thái hoạt động

### 💡 Kịch bản sử dụng
```
[STM32 Sensor] --BLE--> [ESP32 Gateway] --WiFi/MQTT--> [ThingsBoard Cloud]
     (Đọc cảm biến)      (Chuyển tiếp)              (Hiển thị/điều khiển)
```

---

## 2. KIẾN THỨC CƠ BẢN CẦN THIẾT

### 2.1. 📡 BLE (Bluetooth Low Energy)

#### Khái niệm
- **BLE** là phiên bản tiết kiệm năng lượng của Bluetooth
- Phù hợp cho các thiết bị IoT chạy pin
- Phạm vi hoạt động: 10-50m

#### Vai trò trong BLE
- **Peripheral (Slave)**: Thiết bị quảng bá dữ liệu (trong dự án này là **ENV_SENSOR**)
- **Central (Master)**: Thiết bị quét và kết nối (trong dự án này là **ESP32**)

#### Quy trình kết nối BLE
```
1. Peripheral broadcast (quảng bá) → Advertising packets
2. Central scan (quét) → Tìm thiết bị theo tên/UUID
3. Central connect → Thiết lập kết nối
4. Service discovery → Tìm các service/characteristic
5. Data exchange → Đọc/ghi/nhận notification
```

### 2.2. 🔐 GATT (Generic Attribute Profile)

#### Cấu trúc GATT
```
Device (ENV_SENSOR)
└── Service (UUID: 0x00FF)
    └── Characteristic (UUID: 0xFF01)
        ├── Properties: READ, NOTIFY
        ├── Value: Sensor Data (6 bytes)
        └── Descriptor (CCCD)
            └── Enable/Disable Notification
```

#### Thuật ngữ GATT
- **Service**: Nhóm các chức năng liên quan (ví dụ: Environmental Sensing Service)
- **Characteristic**: Điểm dữ liệu cụ thể (ví dụ: nhiệt độ, độ ẩm)
- **Descriptor**: Metadata về characteristic (ví dụ: bật/tắt notification)
- **UUID**: Định danh duy nhất cho service/characteristic
  - **UUID16**: 2 bytes (0x00FF, 0xFF01) - Custom
  - **UUID128**: 16 bytes - Standard Bluetooth SIG

#### Notification vs Indication
- **Notification**: Server gửi → Client nhận (không cần ACK)
- **Indication**: Server gửi → Client phải trả lời ACK
- Dự án này dùng **Notification** để tiết kiệm năng lượng

### 2.3. 📡 MQTT (Message Queuing Telemetry Transport)

#### Khái niệm
- Giao thức **publish/subscribe** nhẹ cho IoT
- Client kết nối tới **Broker** (ThingsBoard) để gửi/nhận dữ liệu

#### Thành phần MQTT
```
[ESP32 Client] --CONNECT--> [ThingsBoard Broker:1883]
      |                              |
      +--PUBLISH--> topic: v1/devices/me/telemetry
      |                              |
      +--SUBSCRIBE--> topic: v1/devices/me/rpc/request/+
      |                              |
      <--PUBLISH-- topic: v1/devices/me/rpc/request/123
```

#### Topics trong ThingsBoard Device API
- `v1/devices/me/telemetry`: Gửi dữ liệu cảm biến (temperature, humidity)
- `v1/devices/me/attributes`: Gửi thuộc tính thiết bị (MAC address, firmware version)
- `v1/devices/me/rpc/request/+`: Nhận lệnh điều khiển từ server
- `v1/devices/me/rpc/response/<request_id>`: Phản hồi kết quả RPC

#### QoS (Quality of Service)
- **QoS 0**: At most once (gửi 1 lần, không đảm bảo)
- **QoS 1**: At least once (đảm bảo nhận, có thể trùng lặp)
- **QoS 2**: Exactly once (đảm bảo nhận đúng 1 lần)
- Dự án này dùng **QoS 1** cho telemetry

### 2.4. ☁️ ThingsBoard

#### Khái niệm
- **IoT Platform** mã nguồn mở để quản lý thiết bị, dữ liệu, dashboard
- Hỗ trợ nhiều giao thức: MQTT, HTTP, CoAP

#### Device API
```json
// Publish telemetry
Topic: v1/devices/me/telemetry
Payload: {"temperature":25.5,"humidity":60.2}

// Publish attributes
Topic: v1/devices/me/attributes
Payload: {"esp32_mac":"AA:BB:CC:DD:EE:FF"}

// RPC Request (từ server)
Topic: v1/devices/me/rpc/request/123
Payload: {"method":"setLED","params":{"state":1}}

// RPC Response (từ device)
Topic: v1/devices/me/rpc/response/123
Payload: {"success":true}
```

#### Access Token
- Mỗi thiết bị có **Device Token** duy nhất
- Token dùng làm **MQTT username** để xác thực
- Ví dụ: `QDIgvpcnumAtFmIJ1GsW`

### 2.5. 🧵 FreeRTOS (Real-Time Operating System)

#### Khái niệm
- Hệ điều hành thời gian thực cho ESP32
- Quản lý đa nhiệm (multitasking) trên vi điều khiển

#### Các khái niệm cơ bản
- **Task**: Một luồng xử lý độc lập (tương tự thread)
- **Event Group**: Cơ chế đồng bộ hóa giữa các task
- **Delay**: `vTaskDelay(pdMS_TO_TICKS(2000))` - delay 2 giây

---

## 3. KIẾN TRÚC HỆ THỐNG

### 3.1. Sơ đồ khối tổng quan

```
┌─────────────────────────────────────────────────────────────┐
│                      ESP32 GATEWAY                          │
│                                                             │
│  ┌──────────────┐         ┌──────────────┐                │
│  │  BLE Stack   │         │  WiFi Stack  │                │
│  │              │         │              │                │
│  │  - Scanner   │         │  - Station   │                │
│  │  - GATT      │         │  - TCP/IP    │                │
│  │    Client    │         │              │                │
│  └──────┬───────┘         └──────┬───────┘                │
│         │                        │                         │
│         ▼                        ▼                         │
│  ┌────────────────────────────────────┐                   │
│  │      main_gateway.c                │                   │
│  │  - BLE connection logic            │                   │
│  │  - Sensor data parsing             │                   │
│  │  - LED control                     │                   │
│  └──────────────┬─────────────────────┘                   │
│                 │                                          │
│                 ▼                                          │
│  ┌────────────────────────────────────┐                   │
│  │      thingsboard.c                 │                   │
│  │  - WiFi connection                 │                   │
│  │  - MQTT client                     │                   │
│  │  - RPC handler                     │                   │
│  └────────────────────────────────────┘                   │
└─────────────────────────────────────────────────────────────┘
         │                               │
         │ BLE                           │ WiFi/MQTT
         ▼                               ▼
┌─────────────────┐            ┌──────────────────┐
│  ENV_SENSOR     │            │  ThingsBoard     │
│  (STM32)        │            │  Cloud           │
│                 │            │                  │
│  - Temperature  │            │  - Dashboard     │
│  - Humidity     │            │  - Alarm         │
│  - LED State    │            │  - RPC Control   │
└─────────────────┘            └──────────────────┘
```

### 3.2. Luồng dữ liệu

```
[ENV_SENSOR] → BLE Notification (6 bytes)
       ↓
[ESP32] parse_sensor_data()
       ↓
   sensor_data_t {temperature, humidity, led_state}
       ↓
[ESP32] thingsboard_send_data()
       ↓
   MQTT Publish → {"temperature":25.5,"humidity":60.2,"led_state":1}
       ↓
[ThingsBoard] Store & Display
```

### 3.3. RPC Control Flow

```
[ThingsBoard Dashboard] User clicks "Set LED"
       ↓
   MQTT Publish → v1/devices/me/rpc/request/123
       ↓
   {"method":"setLED","params":{"state":1}}
       ↓
[ESP32] mqtt_event_handler() → extract request_id
       ↓
[ESP32] rpc_callback("setLED", "{\"state\":1}", 123)
       ↓
[ESP32] Control GPIO/send BLE command
       ↓
[ESP32] MQTT Publish → v1/devices/me/rpc/response/123
       ↓
   {"success":true}
       ↓
[ThingsBoard] Display result
```

---

## 4. CẤU TRÚC CODE

### 4.1. Tệp tin chính

```
esp32_ble_gateway/
├── main/
│   ├── main_gateway.c         ← Logic BLE và điều khiển chính
│   ├── thingsboard.c          ← WiFi & MQTT integration
│   ├── include/
│   │   ├── thingsboard.h      ← API declarations
│   │   └── thingsboard_config.h ← WiFi/MQTT credentials
│   ├── CMakeLists.txt
│   └── Kconfig.projbuild
├── build/                     ← Compiled binaries
├── partitions.csv             ← Flash memory layout
├── sdkconfig                  ← ESP-IDF configuration
└── README.md
```

### 4.2. Module main_gateway.c

#### Cấu trúc dữ liệu chính
```c
// Profile quản lý GATT client
struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;        // Callback function
    uint16_t gattc_if;              // GATT interface
    uint16_t conn_id;               // Connection ID
    uint16_t service_start_handle;  // Service handle range
    uint16_t service_end_handle;
    uint16_t char_handle;           // Characteristic handle
    esp_bd_addr_t remote_bda;       // Device MAC address
};
```

#### Các hàm quan trọng
- `app_main()`: Entry point, khởi tạo hệ thống
- `esp_gap_cb()`: Xử lý sự kiện GAP (scan, connect)
- `esp_gattc_cb()`: Dispatcher cho GATT events
- `gattc_profile_event_handler()`: Xử lý logic GATT client
- `parse_sensor_data()`: Parse dữ liệu 6 bytes từ BLE
- `led_init()`: Khởi tạo LED GPIO

### 4.3. Module thingsboard.c

#### API công khai
```c
void thingsboard_wifi_init(void);              // Kết nối WiFi
void thingsboard_mqtt_init(void);              // Kết nối MQTT
void thingsboard_send_data(sensor_data_t*);    // Gửi telemetry
bool thingsboard_is_connected(void);           // Kiểm tra kết nối
void thingsboard_register_rpc_callback(...);   // Đăng ký RPC handler
```

#### Event handlers
- `wifi_event_handler()`: Xử lý WiFi connect/disconnect
- `mqtt_event_handler()`: Xử lý MQTT connect/data/RPC

---

## 5. LUỒNG HOẠT ĐỘNG CHI TIẾT

### 5.1. 🚀 Khởi động hệ thống (app_main)

```
┌─────────────────────────────────────────┐
│  1. NVS Flash init                      │ ← Lưu trữ cấu hình
│     └─ nvs_flash_init()                 │
├─────────────────────────────────────────┤
│  2. LED init                            │ ← GPIO2 output
│     └─ led_init()                       │
├─────────────────────────────────────────┤
│  3. WiFi init                           │ ← Kết nối WiFi
│     └─ thingsboard_wifi_init()          │
│        ├─ esp_netif_init()              │
│        ├─ esp_wifi_init()               │
│        ├─ esp_wifi_set_config()         │
│        ├─ esp_wifi_start()              │
│        └─ xEventGroupWaitBits()         │ ← Chờ kết nối
├─────────────────────────────────────────┤
│  4. MQTT init                           │ ← Kết nối ThingsBoard
│     └─ thingsboard_mqtt_init()          │
│        ├─ esp_mqtt_client_init()        │
│        └─ esp_mqtt_client_start()       │
├─────────────────────────────────────────┤
│  5. Wait MQTT connection                │ ← Delay 2s
│     └─ vTaskDelay(2000ms)               │
├─────────────────────────────────────────┤
│  6. BLE init                            │ ← Khởi tạo BLE
│     ├─ esp_bt_controller_init()         │
│     ├─ esp_bt_controller_enable()       │
│     ├─ esp_bluedroid_init()             │
│     ├─ esp_bluedroid_enable()           │
│     ├─ esp_ble_gap_register_callback()  │
│     ├─ esp_ble_gattc_register_callback()│
│     └─ esp_ble_gattc_app_register()     │
└─────────────────────────────────────────┘
```

### 5.2. 🔍 BLE Scanning & Connection

#### Step 1: Start Scanning
```
ESP_GATTC_REG_EVT (GATT client registered)
  ↓
esp_ble_gap_set_scan_params(&ble_scan_params)
  ↓
ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT
  ↓
esp_ble_gap_start_scanning(30 seconds)
  ↓
ESP_GAP_BLE_SCAN_START_COMPLETE_EVT
  → "Scan started successfully"
```

#### Step 2: Device Discovery
```
ESP_GAP_BLE_SCAN_RESULT_EVT (foreach advertising packet)
  ↓
Extract device name from adv_data
  ↓
if (strcmp(device_name, "ENV_SENSOR") == 0)
  ↓
  esp_ble_gap_stop_scan()
  ↓
  esp_ble_gattc_open(gattc_if, remote_bda, ...)
    → "Connecting to ENV_SENSOR..."
```

#### Step 3: Connection Established
```
ESP_GATTC_CONNECT_EVT
  ↓
Save: conn_id, remote_bda
  ↓
esp_ble_gattc_search_service(conn_id, &filter_uuid)
  → "Searching for service 0x00FF..."
```

### 5.3. 🔎 Service Discovery

```
ESP_GATTC_SEARCH_RES_EVT (foreach service found)
  ↓
if (service_uuid == 0x00FF)
  ↓
  Save: service_start_handle, service_end_handle
  ↓
  get_server = true
  
ESP_GATTC_SEARCH_CMPL_EVT (discovery complete)
  ↓
if (get_server == true)
  ↓
  Find characteristic 0xFF01:
    esp_ble_gattc_get_attr_count(conn_id, ...)
    esp_ble_gattc_get_char_by_uuid(conn_id, ...)
  ↓
  Save: char_handle
  ↓
  Register for notification:
    esp_ble_gattc_register_for_notify(gattc_if, remote_bda, char_handle)
```

### 5.4. 📡 Enable Notification

```
ESP_GATTC_REG_FOR_NOTIFY_EVT
  ↓
if (status == ESP_GATT_OK)
  ↓
  Find CCCD (Client Characteristic Configuration Descriptor):
    esp_ble_gattc_get_descr_by_char_handle(conn_id, char_handle, ...)
  ↓
  Write to CCCD to enable notification:
    uint16_t notify_en = 0x0001;  // Enable notification
    esp_ble_gattc_write_char_descr(conn_id, descr_handle, &notify_en, ...)
  
ESP_GATTC_WRITE_DESCR_EVT
  ↓
if (status == ESP_GATT_OK)
  → "Notification enabled, waiting for data..."
```

### 5.5. 📊 Receive & Process Data

```
ESP_GATTC_NOTIFY_EVT (every time sensor sends data)
  ↓
Raw data: [0x09, 0xC4, 0x17, 0x70, 0x01, 0x01] (6 bytes)
  ↓
parse_sensor_data(data, len, &sensor_data)
  ↓
  Parse:
    temperature = ((data[0] << 8) | data[1]) / 100.0
                = (0x09C4) / 100.0 = 2500 / 100.0 = 25.0°C
    humidity    = ((data[2] << 8) | data[3]) / 100.0
                = (0x1770) / 100.0 = 6000 / 100.0 = 60.0%
    led_state   = data[4] = 0x01
    is_valid    = data[5] = 0x01
  ↓
if (is_valid)
  ↓
  ESP_LOGI: "Temp: 25.0°C, Humidity: 60.0%, STM32_LED: ON"
  ↓
  thingsboard_send_data(&sensor_data)
```

### 5.6. ☁️ Send to ThingsBoard

```
thingsboard_send_data(sensor_data)
  ↓
if (!mqtt_connected)
  → "MQTT not connected, skipping"
  
if (mqtt_connected)
  ↓
  Create JSON payload:
    {
      "temperature": 25.0,
      "humidity": 60.0,
      "led_state": 1
    }
  ↓
  esp_mqtt_client_publish(
    client,
    topic: "v1/devices/me/telemetry",
    payload: json_string,
    qos: 1
  )
  ↓
  ESP_LOGI: "Sent: T=25.0°C, H=60.0%, LED=1"
```

### 5.7. 🎛️ RPC Command Processing

```
[ThingsBoard] Publish RPC request
  ↓
Topic: v1/devices/me/rpc/request/123
Payload: {"method":"setLED","params":{"state":1}}
  ↓
[ESP32] MQTT_EVENT_DATA
  ↓
mqtt_event_handler()
  ↓
  Extract request_id from topic: 123
  ↓
  Parse JSON:
    method = "setLED"
    params = "{\"state\":1}"
  ↓
  Call rpc_callback("setLED", params, 123)
    ↓ (user implementation)
    Control GPIO/send BLE command
    return true/false
  ↓
  Publish response:
    Topic: v1/devices/me/rpc/response/123
    Payload: {"success":true}
```

---

## 6. GIAO THỨC DỮ LIỆU

### 6.1. 📡 BLE Data Format (6 bytes)

```
Byte 0-1: Temperature (int16_t, big-endian, unit: 0.01°C)
Byte 2-3: Humidity (uint16_t, big-endian, unit: 0.01%)
Byte 4:   STM32 LED state (0=OFF, 1=ON)
Byte 5:   Valid flag (0=invalid, 1=valid)
```

#### Ví dụ:
```
Raw data: [0x09, 0xC4, 0x17, 0x70, 0x01, 0x01]

Temperature:
  0x09C4 = 2500 decimal
  2500 / 100.0 = 25.0°C

Humidity:
  0x1770 = 6000 decimal
  6000 / 100.0 = 60.0%

LED State: 0x01 = ON
Valid: 0x01 = Valid
```

### 6.2. ☁️ MQTT Telemetry Format

```json
Topic: v1/devices/me/telemetry
QoS: 1
Payload:
{
  "temperature": 25.0,    // °C (float)
  "humidity": 60.0,       // % (float)
  "led_state": 1          // 0 or 1 (int)
}
```

### 6.3. 🔧 MQTT RPC Format

#### Request (from ThingsBoard → ESP32)
```json
Topic: v1/devices/me/rpc/request/<request_id>
Payload:
{
  "method": "setLED",
  "params": {
    "state": 1
  }
}
```

#### Response (from ESP32 → ThingsBoard)
```json
Topic: v1/devices/me/rpc/response/<request_id>
Payload:
{
  "success": true
}
```

---

## 7. CẤU HÌNH VÀ TRIỂN KHAI

### 7.1. 🔧 Cấu hình WiFi & ThingsBoard

Chỉnh sửa file `main/include/thingsboard_config.h`:

```c
// WiFi Configuration (⚠️ ESP32 chỉ hỗ trợ 2.4GHz!)
#define WIFI_SSID      "Your_WiFi_Name"
#define WIFI_PASS      "your_password"  // Để trống nếu Open WiFi

// ThingsBoard Configuration
#define THINGSBOARD_SERVER   "thingsboard.cloud"  // hoặc IP server của bạn
#define THINGSBOARD_PORT     1883
#define THINGSBOARD_TOKEN    "YOUR_DEVICE_TOKEN"  // Lấy từ ThingsBoard
```

### 7.2. 📱 Tạo Device trên ThingsBoard

1. Đăng nhập vào [ThingsBoard Dashboard](https://thingsboard.cloud)
2. Vào **Devices** → **Add Device**
3. Đặt tên: `ESP32_Gateway`
4. Copy **Access Token** và paste vào `THINGSBOARD_TOKEN`

### 7.3. 🛠️ Build & Flash

```bash
# Set target (ESP32/ESP32-C3/ESP32-S3)
idf.py set-target esp32

# Configure project (optional)
idf.py menuconfig

# Build project
idf.py build

# Flash to device
idf.py -p COM3 flash monitor

# Hoặc chỉ monitor
idf.py -p COM3 monitor
```

### 7.4. 📊 Xem log

```bash
# Monitor serial output
idf.py -p COM3 monitor

# Thoát monitor: Ctrl+]
```

#### Log mẫu khi chạy thành công:
```
I (525) ESP32_GATEWAY: ESP32 BLE Gateway for ENV_SENSOR
I (535) THINGSBOARD: Initializing WiFi...
I (2345) THINGSBOARD: Connected to WiFi SSID:Your_WiFi_Name
I (2350) THINGSBOARD: Got IP:192.168.1.100
I (2355) THINGSBOARD: Initializing MQTT...
I (3500) THINGSBOARD: MQTT_EVENT_CONNECTED
I (3505) THINGSBOARD: Subscribed to RPC topic
I (3510) THINGSBOARD: ESP32 MAC: AA:BB:CC:DD:EE:FF
I (4600) ESP32_GATEWAY: GATT client registered, status 0
I (4605) ESP32_GATEWAY: Scan started successfully
I (8750) ESP32_GATEWAY: Found device: ENV_SENSOR
I (8755) ESP32_GATEWAY: Connecting to ENV_SENSOR...
I (9100) ESP32_GATEWAY: Connected to aa:bb:cc:dd:ee:ff
I (9500) ESP32_GATEWAY: Service 0x00FF found
I (9800) ESP32_GATEWAY: Characteristic 0xFF01 found
I (10200) ESP32_GATEWAY: Notification enabled
I (11500) ESP32_GATEWAY: Temp: 25.0°C, Humidity: 60.0%, STM32_LED: ON
I (11505) THINGSBOARD: Sent: T=25.0°C, H=60.0%, LED=1
```

---

## 8. XỬ LÝ LỖI VÀ DEBUG

### 8.1. ❌ Lỗi thường gặp

#### 1. WiFi không kết nối được
```
E (5000) THINGSBOARD: Failed to connect to SSID:Your_WiFi_Name
```

**Nguyên nhân:**
- ESP32 chỉ hỗ trợ **2.4GHz WiFi** (không hỗ trợ 5GHz)
- Sai SSID/password
- Router tắt DHCP

**Giải pháp:**
- Kiểm tra WiFi 2.4GHz
- Xác nhận SSID/password chính xác
- Thử reset router

#### 2. MQTT không kết nối
```
E (8000) THINGSBOARD: MQTT_EVENT_ERROR
```

**Nguyên nhân:**
- Sai Device Token
- Firewall chặn port 1883
- Server ThingsBoard không online

**Giải pháp:**
- Kiểm tra lại `THINGSBOARD_TOKEN`
- Ping `thingsboard.cloud`
- Kiểm tra port 1883 open: `telnet thingsboard.cloud 1883`

#### 3. Không tìm thấy ENV_SENSOR
```
I (30000) ESP32_GATEWAY: Scan timeout, restarting...
```

**Nguyên nhân:**
- STM32 sensor chưa bật
- STM32 không quảng bá tên "ENV_SENSOR"
- Khoảng cách quá xa

**Giải pháp:**
- Reset STM32 sensor
- Kiểm tra advertising name trên STM32
- Đưa thiết bị gần nhau (<5m)

#### 4. Kết nối BLE bị ngắt
```
W (15000) ESP32_GATEWAY: Disconnected, reason: 0x13
```

**Nguyên nhân:**
- 0x13 = Remote user terminated connection
- 0x08 = Connection timeout
- 0x3E = Connection failed to be established

**Giải pháp:**
- Kiểm tra pin STM32
- Kiểm tra signal strength (RSSI)
- Tăng connection interval

### 8.2. 🐞 Debug Tips

#### Tăng log level
```c
// Trong app_main()
esp_log_level_set("*", ESP_LOG_INFO);
esp_log_level_set("ESP32_GATEWAY", ESP_LOG_DEBUG);
esp_log_level_set("THINGSBOARD", ESP_LOG_DEBUG);
```

#### In raw data BLE
```c
// Trong ESP_GATTC_NOTIFY_EVT
ESP_LOG_BUFFER_HEX("BLE_DATA", p_data->notify.value, p_data->notify.value_len);
```

#### Kiểm tra heap memory
```c
ESP_LOGI(TAG, "Free heap: %d bytes", esp_get_free_heap_size());
```

#### Kiểm tra WiFi RSSI
```c
wifi_ap_record_t ap_info;
esp_wifi_sta_get_ap_info(&ap_info);
ESP_LOGI(TAG, "WiFi RSSI: %d dBm", ap_info.rssi);
```

---

## 9. MỞ RỘNG VÀ TÙY CHỈNH

### 9.1. 🔌 Thêm cảm biến mới

#### Bước 1: Cập nhật sensor_data_t
```c
// In thingsboard.h
typedef struct {
    float temperature;
    float humidity;
    float pressure;      // ← NEW
    uint8_t led_state;
    uint8_t is_valid;
} sensor_data_t;
```

#### Bước 2: Parse thêm dữ liệu
```c
// In parse_sensor_data()
sensor_data->pressure = ((data[6] << 8) | data[7]) / 100.0f;
```

#### Bước 3: Gửi lên ThingsBoard
```c
// In thingsboard_send_data()
snprintf(payload, sizeof(payload),
         "{"
         "\"temperature\":%.1f,"
         "\"humidity\":%.1f,"
         "\"pressure\":%.1f,"    // ← NEW
         "\"led_state\":%d"
         "}",
         sensor_data->temperature,
         sensor_data->humidity,
         sensor_data->pressure,   // ← NEW
         sensor_data->led_state);
```

### 9.2. 🎛️ Implement RPC Command

#### Bước 1: Đăng ký callback
```c
// In app_main()
thingsboard_register_rpc_callback(handle_rpc_command);
```

#### Bước 2: Implement handler
```c
bool handle_rpc_command(const char *method, const char *params, int request_id) {
    ESP_LOGI(TAG, "RPC: method=%s, params=%s", method, params);
    
    if (strcmp(method, "setLED") == 0) {
        // Parse params to get state
        char state_str[16];
        if (json_extract_string(params, "state", state_str, sizeof(state_str))) {
            int state = atoi(state_str);
            gpio_set_level(LED_GPIO, state);
            ESP_LOGI(TAG, "LED set to %s", state ? "ON" : "OFF");
            return true;
        }
    }
    
    return false;  // Command not supported
}
```

#### Bước 3: Test từ ThingsBoard
1. Vào **Device** → **RPC** tab
2. Chọn **Two-way RPC**
3. Method: `setLED`
4. Params: `{"state": 1}`
5. Click **Send**

### 9.3. 📡 Gửi lệnh từ ESP32 → STM32 qua BLE

```c
// Ghi dữ liệu vào characteristic
uint8_t cmd_data[] = {0x01, 0x02, 0x03};  // Lệnh tùy chỉnh
esp_ble_gattc_write_char(
    gattc_if,
    conn_id,
    char_handle,
    sizeof(cmd_data),
    cmd_data,
    ESP_GATT_WRITE_TYPE_RSP,  // Chờ ACK
    ESP_GATT_AUTH_REQ_NONE
);
```

### 9.4. 💾 Lưu trữ dữ liệu cục bộ (NVS)

```c
#include "nvs.h"

// Mở NVS namespace
nvs_handle_t nvs_handle;
nvs_open("storage", NVS_READWRITE, &nvs_handle);

// Lưu giá trị
float last_temp = 25.5;
nvs_set_blob(nvs_handle, "last_temp", &last_temp, sizeof(float));
nvs_commit(nvs_handle);

// Đọc giá trị
size_t required_size = sizeof(float);
nvs_get_blob(nvs_handle, "last_temp", &last_temp, &required_size);

// Đóng NVS
nvs_close(nvs_handle);
```

### 9.5. 🔋 Tối ưu năng lượng

```c
// BLE: Tăng connection interval (giảm tần suất truyền)
esp_ble_conn_update_params_t conn_params = {
    .min_int = 0x20,  // 20*1.25ms = 25ms (default: 7.5ms)
    .max_int = 0x40,  // 40*1.25ms = 50ms
    .latency = 0,
    .timeout = 400,   // 4s
};
esp_ble_gap_update_conn_params(&conn_params);

// WiFi: Bật Power Save Mode
esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // Hoặc WIFI_PS_MAX_MODEM
```

---

## 📚 TÀI LIỆU THAM KHẢO

### ESP-IDF Documentation
- [ESP32 BLE GATT Client](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_gattc.html)
- [ESP32 WiFi](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html)
- [ESP-MQTT](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/mqtt.html)

### ThingsBoard Documentation
- [Device API](https://thingsboard.io/docs/reference/mqtt-api/)
- [RPC from Server](https://thingsboard.io/docs/user-guide/rpc/#server-side-rpc)
- [Telemetry Upload](https://thingsboard.io/docs/user-guide/telemetry/)

### Bluetooth Specifications
- [BLE Core Specification](https://www.bluetooth.com/specifications/bluetooth-core-specification/)
- [GATT Profile](https://www.bluetooth.com/specifications/specs/gatt-specification-supplement/)

---

## ❓ FAQ (Câu hỏi thường gặp)

### Q1: ESP32 có thể kết nối với bao nhiêu BLE device?
**A:** ESP32 hỗ trợ tối đa **9 kết nối đồng thời** (cả Central và Peripheral mode).

### Q2: Tại sao ESP32 không kết nối được WiFi 5GHz?
**A:** ESP32 chỉ hỗ trợ **2.4GHz WiFi** (802.11 b/g/n). Muốn dùng 5GHz cần dùng ESP32-S3 với module WiFi 6.

### Q3: MQTT QoS nào tốt nhất cho IoT?
**A:** 
- **QoS 0**: Nhanh nhất, dùng cho dữ liệu không quan trọng
- **QoS 1**: Cân bằng, phù hợp hầu hết trường hợp (được dùng trong project này)
- **QoS 2**: Chậm nhất, chỉ dùng khi dữ liệu rất quan trọng

### Q4: Làm sao để ESP32 tự động reconnect khi mất kết nối?
**A:** Đã implement trong `wifi_event_handler()` và `mqtt_event_handler()`:
- WiFi: Auto retry tối đa 10 lần
- MQTT: ESP-MQTT client tự động reconnect

### Q5: Có thể gửi ảnh/video qua MQTT không?
**A:** Có thể nhưng **không nên**:
- MQTT giới hạn payload ~128KB
- Nên dùng HTTP/FTP để upload file lớn, chỉ gửi URL qua MQTT

### Q6: Làm sao để bảo mật MQTT connection?
**A:** 
1. Dùng **MQTTS** (MQTT over TLS/SSL):
   ```c
   .broker.address.uri = "mqtts://thingsboard.cloud",
   .broker.address.port = 8883,
   .broker.verification.certificate = server_cert_pem_start,
   ```
2. Dùng **X.509 certificates** thay vì Access Token

---

## 🎓 KẾT LUẬN

Dự án này minh họa cách xây dựng một **IoT Gateway hoàn chỉnh** sử dụng ESP32, bao gồm:

✅ **BLE GATT Client** - Kết nối và nhận dữ liệu từ sensor  
✅ **WiFi/MQTT** - Gửi dữ liệu lên cloud  
✅ **ThingsBoard Integration** - Hiển thị và điều khiển từ xa  
✅ **RPC Support** - Nhận lệnh điều khiển từ server  

### Các kỹ năng đã học:
- BLE GATT Protocol & Service Discovery
- MQTT Publish/Subscribe Pattern
- FreeRTOS Task Management
- ESP-IDF Framework
- IoT Cloud Platform Integration

### Hướng phát triển:
- Thêm nhiều cảm biến (CO2, PM2.5, ...)
- Implement OTA (Over-The-Air) firmware update
- Lưu trữ dữ liệu offline khi mất kết nối
- Machine Learning tại Edge (TensorFlow Lite)

---

**📧 Liên hệ:** Nếu có câu hỏi, tạo Issue trên GitHub repository.

**⭐ Hữu ích?** Đừng quên star repository!

**📅 Cập nhật:** 26/01/2026
