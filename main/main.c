#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/touch_sens.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "esp_timer.h"

#define TAG "TOUCH"

/* ---------- Wi-Fi credentials (change as needed) ---------- */
#define WIFI_SSID      "fisciano"
#define WIFI_PASSWORD  "Qwertyuiop1234%"

/* ---------- MQTT broker ---------- */
#define MQTT_BROKER_URI  "ws://193.205.185.56:7080"
#define MQTT_TOPIC_CMD   "/commands/"
#define MQTT_TOPIC_VAL   "/values/"

/* ---------- Output GPIOs ---------- */
#define OUTPUT_COUNT 3
static const gpio_num_t output_gpios[OUTPUT_COUNT] = {
    GPIO_NUM_7,
    GPIO_NUM_8,
    GPIO_NUM_9,
};

/* ---------- Current command (set via MQTT, defaults before first message) ---------- */
static SemaphoreHandle_t s_cmd_mutex;

typedef struct {
    uint8_t  mask[OUTPUT_COUNT];  /* which outputs to drive */
    uint32_t duration_ms;         /* how long outputs stay ON */
    uint32_t pause_ms;            /* mandatory pause after pulse */
} cmd_t;

static cmd_t s_cmd = {
    .mask        = {1, 1, 1},
    .duration_ms = 300,
    .pause_ms    = 500,
};

/* ---------- Pulse task ---------- */
/*
 * The pulse task repeats ON/pause cycles while s_running is true.
 * Notification value 1 = start, 2 = stop.
 */
#define PULSE_NOTIFY_START 1
#define PULSE_NOTIFY_STOP  2

static SemaphoreHandle_t s_pulse_mutex;
static TaskHandle_t      s_pulse_task;

static uint8_t  s_pulse_mask[OUTPUT_COUNT];
static uint32_t s_pulse_duration_ms;
static uint32_t s_pulse_pause_ms;

static void outputs_all_off(void)
{
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        gpio_set_level(output_gpios[i], 0);
    }
}

static void pulse_task(void *arg)
{
    while (1) {
        /* Wait for START notification */
        uint32_t note;
        do {
            xTaskNotifyWait(0, ULONG_MAX, &note, portMAX_DELAY);
        } while (note != PULSE_NOTIFY_START);

        xSemaphoreTake(s_pulse_mutex, portMAX_DELAY);
        uint8_t  mask[OUTPUT_COUNT];
        uint32_t duration_ms = s_pulse_duration_ms;
        uint32_t pause_ms    = s_pulse_pause_ms;
        memcpy(mask, s_pulse_mask, OUTPUT_COUNT);
        xSemaphoreGive(s_pulse_mutex);

        /* Repeat ON/pause until STOP notification */
        while (1) {
            ESP_LOGI(TAG, "OUT ON  [%d %d %d] dur=%" PRIu32 "ms",
                     mask[0], mask[1], mask[2], duration_ms);
            for (int i = 0; i < OUTPUT_COUNT; i++) {
                gpio_set_level(output_gpios[i], mask[i] ? 1 : 0);
            }

            /* Wait for duration_ms or STOP */
            xTaskNotifyWait(0, ULONG_MAX, &note, pdMS_TO_TICKS(duration_ms));
            outputs_all_off();
            ESP_LOGI(TAG, "OUT OFF [0 0 0]");

            if (note == PULSE_NOTIFY_STOP) break;

            /* Pause between pulses (also interruptible) */
            xTaskNotifyWait(0, ULONG_MAX, &note, pdMS_TO_TICKS(pause_ms));
            if (note == PULSE_NOTIFY_STOP) break;
        }
    }
}

static void outputs_init(void)
{
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        gpio_reset_pin(output_gpios[i]);
        gpio_set_direction(output_gpios[i], GPIO_MODE_OUTPUT);
        gpio_set_level(output_gpios[i], 0);
    }

    s_pulse_mutex = xSemaphoreCreateMutex();
    xTaskCreate(pulse_task, "pulse", 4096, NULL, 5, &s_pulse_task);
}

static void outputs_start(const uint8_t mask[OUTPUT_COUNT], uint32_t duration_ms, uint32_t pause_ms)
{
    xSemaphoreTake(s_pulse_mutex, portMAX_DELAY);
    memcpy(s_pulse_mask, mask, OUTPUT_COUNT);
    s_pulse_duration_ms = duration_ms;
    s_pulse_pause_ms    = pause_ms;
    xSemaphoreGive(s_pulse_mutex);

    xTaskNotify(s_pulse_task, PULSE_NOTIFY_START, eSetValueWithOverwrite);
}

static void outputs_stop(void)
{
    xTaskNotify(s_pulse_task, PULSE_NOTIFY_STOP, eSetValueWithOverwrite);
}

/* ---------- Wi-Fi ---------- */
#define WIFI_CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_events;

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "Wi-Fi disconnected, retrying…");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    s_wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));

    wifi_config_t wcfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ---------- MQTT ---------- */
static esp_mqtt_client_handle_t s_mqtt_client;

/* Original JSON payload of the last command (for republish) */
static SemaphoreHandle_t s_last_cmd_json_mutex;
static char s_last_cmd_json[256];

static void mqtt_handle_command(const char *payload, int len)
{
    char buf[256];
    if (len >= (int)sizeof(buf)) len = (int)sizeof(buf) - 1;
    memcpy(buf, payload, len);
    buf[len] = '\0';

    ESP_LOGI(TAG, "MQTT RX [%s]: %s", MQTT_TOPIC_CMD, buf);

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        ESP_LOGW(TAG, "MQTT: invalid JSON: %s", buf);
        return;
    }

    cJSON *jmask = cJSON_GetObjectItem(root, "mask");
    cJSON *jdur  = cJSON_GetObjectItem(root, "duration_ms");
    cJSON *jpause = cJSON_GetObjectItem(root, "pause_ms");

    xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);

    if (cJSON_IsArray(jmask)) {
        for (int i = 0; i < OUTPUT_COUNT && i < cJSON_GetArraySize(jmask); i++) {
            cJSON *item = cJSON_GetArrayItem(jmask, i);
            s_cmd.mask[i] = cJSON_IsNumber(item) ? (uint8_t)item->valueint : 0;
        }
    }
    if (cJSON_IsNumber(jdur))   s_cmd.duration_ms = (uint32_t)jdur->valueint;
    if (cJSON_IsNumber(jpause)) s_cmd.pause_ms    = (uint32_t)jpause->valueint;

    xSemaphoreGive(s_cmd_mutex);

    /* Store original payload for later republish */
    xSemaphoreTake(s_last_cmd_json_mutex, portMAX_DELAY);
    strncpy(s_last_cmd_json, buf, sizeof(s_last_cmd_json) - 1);
    s_last_cmd_json[sizeof(s_last_cmd_json) - 1] = '\0';
    xSemaphoreGive(s_last_cmd_json_mutex);

    ESP_LOGI(TAG, "MQTT cmd: mask=[%d,%d,%d] dur=%" PRIu32 "ms pause=%" PRIu32 "ms",
             s_cmd.mask[0], s_cmd.mask[1], s_cmd.mask[2],
             s_cmd.duration_ms, s_cmd.pause_ms);

    cJSON_Delete(root);
}

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)data;
    switch (id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        esp_mqtt_client_subscribe(s_mqtt_client, MQTT_TOPIC_CMD, 1);
        break;
    case MQTT_EVENT_DATA:
        if (strncmp(event->topic, MQTT_TOPIC_CMD, event->topic_len) == 0) {
            mqtt_handle_command(event->data, event->data_len);
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;
    default:
        break;
    }
}

static void mqtt_init(void)
{
    s_last_cmd_json_mutex = xSemaphoreCreateMutex();
    s_cmd_mutex           = xSemaphoreCreateMutex();
    s_last_cmd_json[0]    = '\0';

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };
    s_mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
}

static void mqtt_publish_result(uint32_t touch_time_ms,
                                const uint8_t mask[OUTPUT_COUNT],
                                uint32_t duration_ms, uint32_t pause_ms)
{
    char cmd_json[256];
    xSemaphoreTake(s_last_cmd_json_mutex, portMAX_DELAY);
    strncpy(cmd_json, s_last_cmd_json, sizeof(cmd_json) - 1);
    cmd_json[sizeof(cmd_json) - 1] = '\0';
    xSemaphoreGive(s_last_cmd_json_mutex);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "touch_time_ms", touch_time_ms);

    cJSON *jmask = cJSON_CreateArray();
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        cJSON_AddItemToArray(jmask, cJSON_CreateNumber(mask[i]));
    }
    cJSON_AddItemToObject(root, "mask", jmask);
    cJSON_AddNumberToObject(root, "duration_ms", duration_ms);
    cJSON_AddNumberToObject(root, "pause_ms", pause_ms);

    if (cmd_json[0] != '\0') {
        cJSON *orig = cJSON_Parse(cmd_json);
        if (orig) {
            cJSON_AddItemToObject(root, "command", orig);
        }
    }

    char *out = cJSON_PrintUnformatted(root);
    if (out) {
        esp_mqtt_client_publish(s_mqtt_client, MQTT_TOPIC_VAL, out, 0, 1, 0);
        ESP_LOGI(TAG, "MQTT pub %s -> %s", MQTT_TOPIC_VAL, out);
        cJSON_free(out);
    }
    cJSON_Delete(root);
}

/* ---------- Touch ---------- */
#define TOUCH_CHANNEL_COUNT 3
static const int touch_chan_ids[TOUCH_CHANNEL_COUNT] = {4, 5, 6};
#define INIT_SCAN_TIMES 3

static touch_sensor_handle_t  s_sens_handle;
static touch_channel_handle_t s_chan_handle[TOUCH_CHANNEL_COUNT];

static void touch_initial_scanning(void)
{
    ESP_ERROR_CHECK(touch_sensor_enable(s_sens_handle));
    for (int i = 0; i < INIT_SCAN_TIMES; i++) {
        ESP_ERROR_CHECK(touch_sensor_trigger_oneshot_scanning(s_sens_handle, 48000));
    }
    ESP_ERROR_CHECK(touch_sensor_disable(s_sens_handle));

    for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
        uint32_t benchmark[TOUCH_SAMPLE_CFG_NUM] = {};
        ESP_ERROR_CHECK(touch_channel_read_data(s_chan_handle[i],
                        TOUCH_CHAN_DATA_TYPE_BENCHMARK, benchmark));
        touch_channel_config_t cfg = {
            .active_thresh    = {48000},
            .charge_speed     = TOUCH_CHARGE_SPEED_7,
            .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        };
        ESP_ERROR_CHECK(touch_sensor_reconfig_channel(s_chan_handle[i], &cfg));
        ESP_LOGI(TAG, "CH%d benchmark=%" PRIu32 " threshold=%" PRIu32,
                 touch_chan_ids[i], benchmark[0], cfg.active_thresh[0]);
    }
}

/* ---------- Main ---------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();

    outputs_init();
    mqtt_init();

    /* Touch sensor setup */
    touch_sensor_sample_config_t sample_cfg[TOUCH_SAMPLE_CFG_NUM] = {
        TOUCH_SENSOR_V2_DEFAULT_SAMPLE_CONFIG(500, TOUCH_VOLT_LIM_L_0V5, TOUCH_VOLT_LIM_H_2V2)
    };
    touch_sensor_config_t sens_cfg =
        TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(TOUCH_SAMPLE_CFG_NUM, sample_cfg);
    ESP_ERROR_CHECK(touch_sensor_new_controller(&sens_cfg, &s_sens_handle));

    touch_channel_config_t chan_cfg = {
        .active_thresh    = {48000},
        .charge_speed     = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
    };
    for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
        ESP_ERROR_CHECK(touch_sensor_new_channel(s_sens_handle, touch_chan_ids[i],
                        &chan_cfg, &s_chan_handle[i]));
    }

    touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    ESP_ERROR_CHECK(touch_sensor_config_filter(s_sens_handle, &filter_cfg));

    touch_initial_scanning();

    ESP_ERROR_CHECK(touch_sensor_enable(s_sens_handle));
    ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(s_sens_handle));
    ESP_LOGI(TAG, "Touch sensor running");

    bool     all_active    = false;
    int64_t  touch_start   = 0;
    uint8_t  last_mask[OUTPUT_COUNT] = {};
    uint32_t last_duration_ms = 0;
    uint32_t last_pause_ms    = 0;

    while (1) {
        uint32_t data[TOUCH_SAMPLE_CFG_NUM] = {};
        bool     ch_active[TOUCH_CHANNEL_COUNT];

        for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
            touch_channel_read_data(s_chan_handle[i], TOUCH_CHAN_DATA_TYPE_SMOOTH, data);
            ch_active[i] = (data[0] > 48000);
            ESP_LOGI(TAG, "CH%d: %s value=%" PRIu32,
                     touch_chan_ids[i],
                     ch_active[i] ? "\033[31mON\033[0m" : "OFF",
                     data[0]);
        }

        /* All channels touched simultaneously? */
        bool all_now = true;
        for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
            if (!ch_active[i]) { all_now = false; break; }
        }

        if (all_now && !all_active) {
            /* Rising edge: all just became active */
            all_active  = true;
            touch_start = esp_timer_get_time();

            xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);
            memcpy(last_mask, s_cmd.mask, OUTPUT_COUNT);
            last_duration_ms = s_cmd.duration_ms;
            last_pause_ms    = s_cmd.pause_ms;
            xSemaphoreGive(s_cmd_mutex);

            ESP_LOGI(TAG, "\033[32mALL ON — starting output cycle\033[0m");
            outputs_start(last_mask, last_duration_ms, last_pause_ms);

        } else if (!all_now && all_active) {
            /* Falling edge: at least one released */
            all_active = false;
            outputs_stop();
            uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - touch_start) / 1000);
            ESP_LOGI(TAG, "\033[32mALL OFF — touch held %" PRIu32 " ms\033[0m", elapsed_ms);
            mqtt_publish_result(elapsed_ms, last_mask, last_duration_ms, last_pause_ms);
        }

        ESP_LOGI(TAG, "\033[32m-------------\033[0m");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
