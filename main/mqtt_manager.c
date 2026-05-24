#include "mqtt_manager.h"
#include <string.h>
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

static SemaphoreHandle_t s_last_json_mutex;
static char              s_last_json[256];

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

    cJSON *jmask  = cJSON_GetObjectItem(root, "mask");
    cJSON *jdur   = cJSON_GetObjectItem(root, "duration_ms");
    cJSON *jpause = cJSON_GetObjectItem(root, "pause_ms");

    xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);
    if (cJSON_IsArray(jmask)) {
        for (int i = 0; i < OUTPUT_COUNT && i < cJSON_GetArraySize(jmask); i++) {
            cJSON *item = cJSON_GetArrayItem(jmask, i);
            s_cmd->mask[i] = cJSON_IsNumber(item) ? (uint8_t)item->valueint : 0;
        }
    }
    if (cJSON_IsNumber(jdur))   s_cmd->duration_ms = (uint32_t)jdur->valueint;
    if (cJSON_IsNumber(jpause)) s_cmd->pause_ms    = (uint32_t)jpause->valueint;
    xSemaphoreGive(s_cmd_mutex);

    xSemaphoreTake(s_last_json_mutex, portMAX_DELAY);
    strncpy(s_last_json, buf, sizeof(s_last_json) - 1);
    s_last_json[sizeof(s_last_json) - 1] = '\0';
    xSemaphoreGive(s_last_json_mutex);

    ESP_LOGI(TAG, "cmd: mask=[%d,%d,%d] dur=%" PRIu32 "ms pause=%" PRIu32 "ms",
             s_cmd->mask[0], s_cmd->mask[1], s_cmd->mask[2],
             s_cmd->duration_ms, s_cmd->pause_ms);

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

void mqtt_init(mqtt_cmd_t *cmd, void *cmd_mutex)
{
    s_cmd             = cmd;
    s_cmd_mutex       = (SemaphoreHandle_t)cmd_mutex;
    s_last_json_mutex = xSemaphoreCreateMutex();
    s_last_json[0]    = '\0';

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    s_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);
}

void mqtt_publish_result(uint32_t touch_time_ms,
                         const uint8_t touched[TOUCH_CHANNEL_COUNT],
                         const uint8_t outputs[OUTPUT_COUNT],
                         uint32_t duration_ms, uint32_t pause_ms)
{
    char last_json[256];
    xSemaphoreTake(s_last_json_mutex, portMAX_DELAY);
    strncpy(last_json, s_last_json, sizeof(last_json) - 1);
    last_json[sizeof(last_json) - 1] = '\0';
    xSemaphoreGive(s_last_json_mutex);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "touch_time_ms", touch_time_ms);

    cJSON *jtouched = cJSON_CreateArray();
    for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
        cJSON_AddItemToArray(jtouched, cJSON_CreateNumber(touched[i]));
    }
    cJSON_AddItemToObject(root, "touched", jtouched);

    cJSON *joutputs = cJSON_CreateArray();
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        cJSON_AddItemToArray(joutputs, cJSON_CreateNumber(outputs[i]));
    }
    cJSON_AddItemToObject(root, "outputs", joutputs);

    cJSON_AddNumberToObject(root, "duration_ms", duration_ms);
    cJSON_AddNumberToObject(root, "pause_ms", pause_ms);

    // if (last_json[0] != '\0') {
    //     cJSON *orig = cJSON_Parse(last_json);
    //     if (orig) {
    //         cJSON_AddItemToObject(root, "command", orig);
    //     }
    // }

    char *out = cJSON_PrintUnformatted(root);
    if (out) {
        esp_mqtt_client_publish(s_client, MQTT_TOPIC_VAL, out, 0, 1, 0);
        ESP_LOGI(TAG, "pub %s -> %s", MQTT_TOPIC_VAL, out);
        cJSON_free(out);
    }
    cJSON_Delete(root);
}
