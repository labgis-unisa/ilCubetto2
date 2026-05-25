#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/touch_sens.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

#define TAG "TOUCH"

/* ---------- Output GPIOs ---------- */
static const gpio_num_t output_gpios[OUTPUT_COUNT] = {
    GPIO_NUM_7,
    GPIO_NUM_8,
    GPIO_NUM_9,
};

/* ---------- Current command ---------- */
static SemaphoreHandle_t s_cmd_mutex;
static mqtt_cmd_t s_cmd = {
    .pulse_ms = {0, 0, 0},
    .pause_ms = 2000,
};

/* ---------- Pulse task ---------- */
#define PULSE_NOTIFY_START 1
#define PULSE_NOTIFY_STOP  2

static SemaphoreHandle_t s_pulse_mutex;
static TaskHandle_t      s_pulse_task;

static uint32_t s_pulse_ms[OUTPUT_COUNT]; /* per-output pulse duration; 0 = off */
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
        uint32_t note;
        do {
            xTaskNotifyWait(0, ULONG_MAX, &note, portMAX_DELAY);
        } while (note != PULSE_NOTIFY_START);

        xSemaphoreTake(s_pulse_mutex, portMAX_DELAY);
        uint32_t pulse_ms[OUTPUT_COUNT];
        uint32_t pause_ms = s_pulse_pause_ms;
        memcpy(pulse_ms, s_pulse_ms, sizeof(s_pulse_ms));
        xSemaphoreGive(s_pulse_mutex);

        while (1) {
            ESP_LOGI(TAG, "OUT ON  pulse_ms=[%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]",
                     pulse_ms[0], pulse_ms[1], pulse_ms[2]);

            /* Activate outputs that have a non-zero pulse duration */
            for (int i = 0; i < OUTPUT_COUNT; i++) {
                gpio_set_level(output_gpios[i], pulse_ms[i] > 0 ? 1 : 0);
            }

            /* Turn off each output after its individual pulse duration */
            uint32_t elapsed = 0;
            bool stopped = false;
            while (elapsed < UINT32_MAX) {
                /* Find the next output to turn off */
                uint32_t next_off = UINT32_MAX;
                for (int i = 0; i < OUTPUT_COUNT; i++) {
                    if (pulse_ms[i] > elapsed && pulse_ms[i] < next_off) {
                        next_off = pulse_ms[i];
                    }
                }
                if (next_off == UINT32_MAX) break; /* all already off */

                uint32_t wait = next_off - elapsed;
                xTaskNotifyWait(0, ULONG_MAX, &note, pdMS_TO_TICKS(wait));
                if (note == PULSE_NOTIFY_STOP) { stopped = true; break; }
                elapsed = next_off;

                for (int i = 0; i < OUTPUT_COUNT; i++) {
                    if (pulse_ms[i] <= elapsed) {
                        gpio_set_level(output_gpios[i], 0);
                    }
                }
            }
            outputs_all_off();
            ESP_LOGI(TAG, "OUT OFF [0 0 0]");

            if (stopped) break;

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

static void outputs_start(const uint32_t pulse_ms[OUTPUT_COUNT], uint32_t pause_ms)
{
    xSemaphoreTake(s_pulse_mutex, portMAX_DELAY);
    memcpy(s_pulse_ms, pulse_ms, sizeof(s_pulse_ms));
    s_pulse_pause_ms = pause_ms;
    xSemaphoreGive(s_pulse_mutex);

    xTaskNotify(s_pulse_task, PULSE_NOTIFY_START, eSetValueWithOverwrite);
}

static void outputs_stop(void)
{
    xTaskNotify(s_pulse_task, PULSE_NOTIFY_STOP, eSetValueWithOverwrite);
}

/* ---------- Touch ---------- */
#define TOUCH_CHANNEL_COUNT 3
static const int touch_chan_ids[TOUCH_CHANNEL_COUNT] = {4, 5, 6};
#define INIT_SCAN_TIMES 3

static touch_sensor_handle_t  s_sens_handle;
static touch_channel_handle_t s_chan_handle[TOUCH_CHANNEL_COUNT];
static uint32_t s_channel_thresh[TOUCH_CHANNEL_COUNT] = {};

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
        /* Compute adaptive threshold from benchmark to avoid fixed literals */
        uint32_t thresh = benchmark[0] + (benchmark[0] / 8); /* +12.5% margin */
        if (thresh < 1000) thresh = 1000;
        if (thresh > 60000) thresh = 60000;
        touch_channel_config_t cfg = {
            .active_thresh    = {thresh},
            .charge_speed     = TOUCH_CHARGE_SPEED_7,
            .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        };
        ESP_ERROR_CHECK(touch_sensor_reconfig_channel(s_chan_handle[i], &cfg));
        s_channel_thresh[i] = cfg.active_thresh[0];
        ESP_LOGI(TAG, "CH%d benchmark=%" PRIu32 " threshold=%" PRIu32,
                 touch_chan_ids[i], benchmark[0], s_channel_thresh[i]);
    }
}

/* ---------- Main ---------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init();

    outputs_init();

    s_cmd_mutex = xSemaphoreCreateMutex();
    mqtt_init(&s_cmd, s_cmd_mutex);

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

    bool     all_active           = false;
    int64_t  touch_start          = 0;
    uint32_t last_pulse_ms[OUTPUT_COUNT] = {};
    uint32_t last_pause_ms        = 0;

    while (1) {
        uint32_t data[TOUCH_SAMPLE_CFG_NUM] = {};
        bool     ch_active[TOUCH_CHANNEL_COUNT];

        for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
            touch_channel_read_data(s_chan_handle[i], TOUCH_CHAN_DATA_TYPE_SMOOTH, data);
            ch_active[i] = (data[0] > s_channel_thresh[i]);
            ESP_LOGI(TAG, "CH%d: %s value=%" PRIu32 " thresh=%" PRIu32,
                     touch_chan_ids[i],
                     ch_active[i] ? "\033[31mON\033[0m" : "OFF",
                     data[0], s_channel_thresh[i]);
        }

        bool all_now = true;
        for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
            if (!ch_active[i]) { all_now = false; break; }
        }

        if (all_now && !all_active) {
            all_active  = true;
            touch_start = esp_timer_get_time();

            xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);
            memcpy(last_pulse_ms, s_cmd.pulse_ms, sizeof(s_cmd.pulse_ms));
            last_pause_ms = s_cmd.pause_ms;
            xSemaphoreGive(s_cmd_mutex);

            ESP_LOGI(TAG, "\033[32mALL ON — pulse_ms=[%" PRIu32 ",%" PRIu32 ",%" PRIu32 "] pause=%" PRIu32 "ms\033[0m",
                     last_pulse_ms[0], last_pulse_ms[1], last_pulse_ms[2], last_pause_ms);
            outputs_start(last_pulse_ms, last_pause_ms);

        } else if (!all_now && all_active) {
            all_active = false;
            outputs_stop();
            uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - touch_start) / 1000);
            ESP_LOGI(TAG, "\033[32mALL OFF — touch held %" PRIu32 " ms\033[0m", elapsed_ms);

            uint8_t touched[TOUCH_CHANNEL_COUNT];
            for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
                touched[i] = ch_active[i] ? 1 : 0;
            }
            mqtt_publish_result(elapsed_ms, touched, last_pulse_ms, last_pause_ms);
        }

        ESP_LOGI(TAG, "\033[32m-------------\033[0m");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
