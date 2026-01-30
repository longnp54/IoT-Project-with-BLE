/*
 * ThingsBoard Integration Module
 * Handles WiFi connection and MQTT communication with ThingsBoard
 */

#include "thingsboard.h"
#include "thingsboard_config.h"

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define TB_TAG "THINGSBOARD"

// WiFi Event Group
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_RETRY          10

// Static variables
static EventGroupHandle_t s_wifi_event_group;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
static int s_retry_num = 0;
static thingsboard_rpc_callback_t rpc_callback = NULL;

// Helper function to extract request_id from topic
static int extract_request_id(const char *topic) {
    if (topic == NULL) return -1;
    const char *last_slash = strrchr(topic, '/');
    if (last_slash == NULL) return -1;
    return atoi(last_slash + 1);
}

// Helper function to parse JSON and extract string value
static bool json_extract_string(const char *json, const char *key, char *output, size_t output_size) {
    if (json == NULL || key == NULL || output == NULL) return false;
    
    char search_key[64];
    snprintf(search_key, sizeof(search_key), "\"%s\":", key);
    const char *key_pos = strstr(json, search_key);
    if (key_pos == NULL) return false;
    
    const char *value_start = key_pos + strlen(search_key);
    while (*value_start == ' ' || *value_start == '\t') value_start++;
    
    if (*value_start == '\"') {
        // String value
        value_start++;
        const char *value_end = strchr(value_start, '\"');
        if (value_end == NULL) return false;
        size_t len = value_end - value_start;
        if (len >= output_size) len = output_size - 1;
        memcpy(output, value_start, len);
        output[len] = '\0';
    } else {
        // Number or boolean
        const char *value_end = value_start;
        while (*value_end && *value_end != ',' && *value_end != '}' && *value_end != ' ') value_end++;
        size_t len = value_end - value_start;
        if (len >= output_size) len = output_size - 1;
        memcpy(output, value_start, len);
        output[len] = '\0';
    }
    return true;
}

// WiFi Event Handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TB_TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TB_TAG, "Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TB_TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// MQTT Event Handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TB_TAG, "MQTT_EVENT_CONNECTED");
        mqtt_connected = true;
        
        // Subscribe to RPC topic
        int msg_id = esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_RPC_REQUEST, 1);
        if (msg_id >= 0) {
            ESP_LOGI(TB_TAG, "Subscribed to RPC topic");
        } else {
            ESP_LOGE(TB_TAG, "Failed to subscribe to RPC topic");
        }
        
        // Gửi ESP32 MAC address để nhận diện device
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        char attributes[128];
        snprintf(attributes, sizeof(attributes),
                "{\"esp32_mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\"}",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_ATTRIBUTES, attributes, 0, 1, 0);
        ESP_LOGI(TB_TAG, "ESP32 MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        break;
        
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TB_TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = false;
        break;
        
    case MQTT_EVENT_DATA:
        ESP_LOGI(TB_TAG, "MQTT_EVENT_DATA");
        
        // Check if this is an RPC request
        if (event->topic_len > 0 && strncmp(event->topic, "v1/devices/me/rpc/request/", 26) == 0) {
            // Extract request_id from topic
            int request_id = extract_request_id(event->topic);
            if (request_id < 0) {
                ESP_LOGW(TB_TAG, "Invalid RPC request_id");
                break;
            }
            
            // Ensure data is null-terminated
            char data_buffer[256];
            int data_len = event->data_len < (sizeof(data_buffer) - 1) ? event->data_len : (sizeof(data_buffer) - 1);
            memcpy(data_buffer, event->data, data_len);
            data_buffer[data_len] = '\0';
            
            ESP_LOGI(TB_TAG, "RPC Request [%d]: %s", request_id, data_buffer);
            
            // Extract method and params
            char method[32] = {0};
            if (!json_extract_string(data_buffer, "method", method, sizeof(method))) {
                ESP_LOGW(TB_TAG, "Failed to extract method from RPC");
                break;
            }
            
            // Find params object
            const char *params_start = strstr(data_buffer, "\"params\":");
            char params[128] = {0};
            if (params_start != NULL) {
                params_start += 9; // Skip "params":
                while (*params_start == ' ' || *params_start == '\t') params_start++;
                if (*params_start == '{') {
                    const char *params_end = strchr(params_start, '}');
                    if (params_end != NULL) {
                        size_t params_len = params_end - params_start + 1;
                        if (params_len < sizeof(params)) {
                            memcpy(params, params_start, params_len);
                            params[params_len] = '\0';
                        }
                    }
                }
            }
            
            // Call user callback
            bool success = false;
            if (rpc_callback != NULL) {
                success = rpc_callback(method, params, request_id);
            } else {
                ESP_LOGW(TB_TAG, "No RPC callback registered");
            }
            
            // Send response
            char response_topic[64];
            snprintf(response_topic, sizeof(response_topic), "%s%d", MQTT_TOPIC_RPC_RESPONSE, request_id);
            
            char response[64];
            snprintf(response, sizeof(response), "{\"success\":%s}", success ? "true" : "false");
            esp_mqtt_client_publish(mqtt_client, response_topic, response, 0, 1, 0);
            
            ESP_LOGI(TB_TAG, "RPC Response [%d]: %s", request_id, success ? "SUCCESS" : "FAILED");
        }
        break;
        
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TB_TAG, "MQTT_EVENT_ERROR");
        break;
        
    default:
        break;
    }
}

// Initialize WiFi
void thingsboard_wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_OPEN,  // Open WiFi (không mật khẩu)
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TB_TAG, "WiFi initialization finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TB_TAG, "Connected to WiFi SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TB_TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TB_TAG, "UNEXPECTED EVENT");
    }
}

// Initialize MQTT
void thingsboard_mqtt_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://" THINGSBOARD_SERVER,
        .broker.address.port = THINGSBOARD_PORT,
        .credentials.username = THINGSBOARD_TOKEN,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    
    ESP_LOGI(TB_TAG, "MQTT client started");
}

// Send data to ThingsBoard (Device API - đơn giản)
void thingsboard_send_data(sensor_data_t *sensor_data)
{
    ESP_LOGI(TB_TAG, "[SEND] thingsboard_send_data() called");
    
    if (!mqtt_connected || mqtt_client == NULL) {
        ESP_LOGW(TB_TAG, "[SEND] MQTT not connected (mqtt_connected=%d, mqtt_client=%p), skipping send", 
                 mqtt_connected, mqtt_client);
        return;
    }

    if (sensor_data == NULL) {
        ESP_LOGW(TB_TAG, "[SEND] sensor_data is NULL, skipping send");
        return;
    }
    
    if (!sensor_data->is_valid) {
        ESP_LOGW(TB_TAG, "[SEND] Data invalid (is_valid=%d), skipping send", sensor_data->is_valid);
        return;
    }
    
    ESP_LOGI(TB_TAG, "[SEND] Preparing payload: T=%.1f°C, H=%.1f%%, LED=%d",
             sensor_data->temperature, sensor_data->humidity, sensor_data->led_state);

    // Tạo JSON đơn giản: {"temperature":25.5,"humidity":60.2,"led_state":1}
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{"
             "\"temperature\":%.1f,"
             "\"humidity\":%.1f,"
             "\"led_state\":%d"
             "}",
             sensor_data->temperature,
             sensor_data->humidity,
             sensor_data->led_state);

    // Gửi tới Device API topic
    int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_TELEMETRY, payload, 0, 1, 0);
    
    if (msg_id >= 0) {
        ESP_LOGI(TB_TAG, "Sent: T=%.1f°C, H=%.1f%%, LED=%d", 
                 sensor_data->temperature,
                 sensor_data->humidity,
                 sensor_data->led_state);
    } else {
        ESP_LOGE(TB_TAG, "Failed to publish to ThingsBoard");
    }
}

// Check MQTT connection status
bool thingsboard_is_connected(void)
{
    return mqtt_connected;
}

// Register RPC callback function
void thingsboard_register_rpc_callback(thingsboard_rpc_callback_t callback)
{
    if (callback == NULL) {
        ESP_LOGW(TB_TAG, "Trying to register NULL RPC callback");
        return;
    }
    rpc_callback = callback;
    ESP_LOGI(TB_TAG, "RPC callback registered successfully");
}
