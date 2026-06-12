#include "mqtt_manager.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"

#define TAG "MQTT"

#define MQTT_BROKER_URI  "ws://193.205.185.56:7080"

static esp_mqtt_client_handle_t s_client;

static mqtt_cmd_t       *s_cmd;
static SemaphoreHandle_t s_cmd_mutex;
static bool             *s_is_new_cmd_ptr;

static void handle_command(const char *payload, int len)
{
    char buf[256];
    if (len >= (int)sizeof(buf)) len = (int)sizeof(buf) - 1;
    memcpy(buf, payload, len);
    buf[len] = '\0';

    ESP_LOGI(TAG, "RX [%s]: %s", MQTT_TOPIC_CMD, buf);

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        ESP_LOGW(TAG, "invalid JSON: %s", buf);
        return;
    }

    cJSON *jsetpoint  = cJSON_GetObjectItem(root, "temp_setpoint_c");
    cJSON *jchannels  = cJSON_GetObjectItem(root, "channels");

    xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);
    if (cJSON_IsNumber(jsetpoint)) {
        s_cmd->temp_setpoint_c = (float)jsetpoint->valuedouble;
    }
    if (cJSON_IsArray(jchannels)) {
        for (int i = 0; i < OUTPUT_COUNT && i < cJSON_GetArraySize(jchannels); i++) {
            cJSON *item = cJSON_GetArrayItem(jchannels, i);
            s_cmd->channel_enabled[i] = cJSON_IsNumber(item) ? (item->valueint != 0) : true;
        }
    }
    ESP_LOGI(TAG, "cmd after update: temp_setpoint_c=%.1f °C  channels=[%d,%d,%d]",
             s_cmd->temp_setpoint_c,
             s_cmd->channel_enabled[0], s_cmd->channel_enabled[1], s_cmd->channel_enabled[2]);
    *s_is_new_cmd_ptr = true;
    xSemaphoreGive(s_cmd_mutex);

    esp_mqtt_client_publish(s_client, MQTT_TOPIC_READY, "{\"status\":\"ready\"}", 0, 1, 0);
    ESP_LOGI(TAG, "pub " MQTT_TOPIC_READY " -> ready");

    cJSON_Delete(root);
}

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)data;
    switch (id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "connected");
        esp_mqtt_client_subscribe(s_client, MQTT_TOPIC_CMD, 1);
        esp_mqtt_client_publish(s_client, MQTT_TOPIC_ONLINE, "{\"status\":\"online\"}", 0, 1, 0);
        ESP_LOGI(TAG, "pub " MQTT_TOPIC_ONLINE " -> online");
        break;
    case MQTT_EVENT_DATA:
        if (strncmp(event->topic, MQTT_TOPIC_CMD, event->topic_len) == 0) {
            handle_command(event->data, event->data_len);
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "disconnected");
        break;
    default:
        break;
    }
}

void mqtt_init(mqtt_cmd_t *cmd, void *cmd_mutex, bool *is_new_cmd_ptr)
{
    s_cmd             = cmd;
    s_cmd_mutex       = (SemaphoreHandle_t)cmd_mutex;
    s_is_new_cmd_ptr  = is_new_cmd_ptr;

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    s_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);
}

void mqtt_publish_result(uint32_t touch_time_ms,
                         const uint8_t  touched[TOUCH_CHANNEL_COUNT],
                         float setpoint_c,
                         const bool     channel_enabled[OUTPUT_COUNT],
                         const float    temps_max_c[NTC_COUNT],
                         const float    temps_avg_c[NTC_COUNT])
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "touch_time_ms", touch_time_ms);

    cJSON *jtouched = cJSON_CreateArray();
    for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
        cJSON_AddItemToArray(jtouched, cJSON_CreateNumber(touched[i]));
    }
    cJSON_AddItemToObject(root, "touched", jtouched);

    cJSON_AddNumberToObject(root, "temp_setpoint_c", roundf(setpoint_c * 10.0f) / 10.0f);

    cJSON *jchannels = cJSON_CreateArray();
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        cJSON_AddItemToArray(jchannels, cJSON_CreateNumber(channel_enabled[i] ? 1 : 0));
    }
    cJSON_AddItemToObject(root, "channels", jchannels);

    cJSON *jmax = cJSON_CreateArray();
    for (int i = 0; i < NTC_COUNT; i++) {
        cJSON_AddItemToArray(jmax, cJSON_CreateNumber(roundf(temps_max_c[i] * 10.0f) / 10.0f));
    }
    cJSON_AddItemToObject(root, "temps_max_c", jmax);

    cJSON *javg = cJSON_CreateArray();
    for (int i = 0; i < NTC_COUNT; i++) {
        cJSON_AddItemToArray(javg, cJSON_CreateNumber(roundf(temps_avg_c[i] * 10.0f) / 10.0f));
    }
    cJSON_AddItemToObject(root, "temps_avg_c", javg);

    char *out = cJSON_PrintUnformatted(root);
    if (out) {
        esp_mqtt_client_publish(s_client, MQTT_TOPIC_VAL, out, 0, 1, 0);
        ESP_LOGI(TAG, "pub %s -> %s", MQTT_TOPIC_VAL, out);
        cJSON_free(out);
    }
    cJSON_Delete(root);
}
