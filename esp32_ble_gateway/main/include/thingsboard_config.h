#ifndef THINGSBOARD_CONFIG_H
#define THINGSBOARD_CONFIG_H

// WiFi Configuration (ESP32 chỉ hỗ trợ 2.4GHz WiFi)
#define WIFI_SSID      "UET-Wifi-Office-Free 2.4Ghz" 
#define WIFI_PASS      ""                       

// ThingsBoard Configuration
#define THINGSBOARD_SERVER   "thingsboard.cloud"  // Hoặc địa chỉ server của bạn
#define THINGSBOARD_PORT     1883
#define THINGSBOARD_TOKEN    "QDIgvpcnumAtFmIJ1GsW"  // Device Token từ ThingsBoard

// MQTT Topics for Device API
#define MQTT_TOPIC_TELEMETRY   "v1/devices/me/telemetry"
#define MQTT_TOPIC_ATTRIBUTES  "v1/devices/me/attributes"
#define MQTT_TOPIC_RPC_REQUEST "v1/devices/me/rpc/request/+"
#define MQTT_TOPIC_RPC_RESPONSE "v1/devices/me/rpc/response/"

#endif // THINGSBOARD_CONFIG_H
