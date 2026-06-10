#pragma once
#include <stdint.h>
#include <stdbool.h>

#define OUTPUT_COUNT 3
#define TOUCH_CHANNEL_COUNT 3

#define MQTT_TOPIC_CMD    "/commands/"
#define MQTT_TOPIC_VAL    "/values/"
#define MQTT_TOPIC_ONLINE "/online/"
#define MQTT_TOPIC_READY  "/ready/"

typedef struct
{
    float temp_setpoint_c;          /* target temperature for the closed-loop controller */
    bool  channel_enabled[OUTPUT_COUNT]; /* which heater channels to activate */
} mqtt_cmd_t;

/* Starts the MQTT client and subscribes to MQTT_TOPIC_CMD. */
void mqtt_init(mqtt_cmd_t *cmd, void *cmd_mutex, bool *is_new_cmd_ptr);

#define NTC_COUNT 3

/* Publishes a result message to MQTT_TOPIC_VAL. */
void mqtt_publish_result(uint32_t touch_time_ms,
                         const uint8_t touched[TOUCH_CHANNEL_COUNT],
                         float setpoint_c,
                         const float temps_c[NTC_COUNT]);
