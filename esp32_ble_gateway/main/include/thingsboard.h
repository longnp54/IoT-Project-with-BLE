#ifndef THINGSBOARD_H
#define THINGSBOARD_H

#include <stdint.h>
#include <stdbool.h>

// Sensor data structure (simplified for Device API)
typedef struct {
    float temperature;
    float humidity;
    uint8_t led_state;
    uint8_t is_valid;
} sensor_data_t;


typedef bool (*thingsboard_rpc_callback_t)(const char *method, const char *params, int request_id);

void thingsboard_wifi_init(void);
void thingsboard_mqtt_init(void);
void thingsboard_send_data(sensor_data_t *sensor_data);
bool thingsboard_is_connected(void);
void thingsboard_register_rpc_callback(thingsboard_rpc_callback_t callback);

#endif // THINGSBOARD_H
