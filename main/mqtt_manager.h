#pragma once
#include <stdint.h>

#define OUTPUT_COUNT        3
#define TOUCH_CHANNEL_COUNT 3

#define MQTT_TOPIC_CMD  "/commands/"
#define MQTT_TOPIC_VAL  "/values/"

typedef struct {
    uint32_t pulse_ms[OUTPUT_COUNT]; /* per-output pulse duration; 0 = off */
    uint32_t pause_ms;
} mqtt_cmd_t;

/* Starts the MQTT client and subscribes to MQTT_TOPIC_CMD. */
void mqtt_init(mqtt_cmd_t *cmd, void *cmd_mutex);

/* Publishes a result message to MQTT_TOPIC_VAL. */
void mqtt_publish_result(uint32_t touch_time_ms,
                         const uint8_t  touched[TOUCH_CHANNEL_COUNT],
                         const uint32_t pulse_ms[OUTPUT_COUNT],
                         uint32_t pause_ms);
