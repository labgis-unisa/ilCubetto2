#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/touch_sens.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_adc/adc_oneshot.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

#define TAG "TOUCH"

/* ---------- NTC temperature ---------- */
/* Voltage divider: 3.3V -- 10K fixed -- GPIOx -- 10K NTC -- GND
   GPIO1 = ADC1_CH0, GPIO2 = ADC1_CH1, GPIO3 = ADC1_CH2 (ESP32-S3) */
#define NTC_COUNT        3
#define NTC_SERIES_R     10000.0f   /* series resistor, ohms */
#define NTC_NOMINAL_R    10000.0f   /* NTC resistance at 25 °C */
#define NTC_NOMINAL_T    25.0f      /* nominal temperature, °C */
#define NTC_BETA         3980.0f    /* NTC B-coefficient - TDK b57421V2103*/
#define ADC_MAX_VAL      4095.0f    /* 12-bit ADC */
#define NTC_OVERSAMPLE   16     /* ADC reads averaged per channel per call */
#define NTC_MIN_VALID_C  15.0f  /* readings below this value are treated as invalid (open circuit / noise) */

static const adc_channel_t s_ntc_channels[NTC_COUNT] = {
    ADC_CHANNEL_1,  /* GPIO2 POLLICE*/
    ADC_CHANNEL_2,  /* GPIO3 INDICE*/
    ADC_CHANNEL_0,  /* GPIO1 MEDIO*/
};

/* Per-channel calibration offsets (°C to subtract), measured vs thermocouple reference */
static const float s_ntc_offset_c[NTC_COUNT] = {
    20.8f,  /* GPIO2 */
    20.8f,  /* GPIO3 */
    20.8f,  /* GPIO1 */
};

static adc_oneshot_unit_handle_t s_adc_handle;

static void ntc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,   /* 0–3.3 V range */
        .bitwidth = ADC_BITWIDTH_12,
    };
    for (int i = 0; i < NTC_COUNT; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, s_ntc_channels[i], &chan_cfg));
    }
}

static float ntc_raw_to_celsius(int raw, float offset_c)
{
    /* Voltage divider: Vout = Vcc * R_series / (NTC + R_series)
       => NTC = R_series * (ADC_MAX - raw) / raw */
    float r_ntc = NTC_SERIES_R * (ADC_MAX_VAL - (float)raw) / (float)raw;

    /* Steinhart-Hart (simplified B-parameter equation):
       1/T = 1/T0 + (1/B) * ln(R/R0)   [T in Kelvin] */
    float t_kelvin = 1.0f / (1.0f / (NTC_NOMINAL_T + 273.15f)
                             + logf(r_ntc / NTC_NOMINAL_R) / NTC_BETA);
    return t_kelvin - 273.15f - offset_c;
}


static float s_ntc_last_valid[NTC_COUNT] = {NTC_MIN_VALID_C, NTC_MIN_VALID_C, NTC_MIN_VALID_C};

static void ntc_read_all(float out_celsius[NTC_COUNT])
{
    for (int i = 0; i < NTC_COUNT; i++) {
        int32_t sum = 0;
        for (int s = 0; s < NTC_OVERSAMPLE; s++) {
            int raw = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(s_adc_handle, s_ntc_channels[i], &raw));
            sum += raw;
        }
        float t = ntc_raw_to_celsius((int)(sum / NTC_OVERSAMPLE), s_ntc_offset_c[i]);
        if (t >= NTC_MIN_VALID_C) {
            s_ntc_last_valid[i] = t;
        } else {
            ESP_LOGW(TAG, "NTC[%d] reading %.1f °C below minimum, using last valid %.1f °C", i, t, s_ntc_last_valid[i]);
        }
        out_celsius[i] = s_ntc_last_valid[i];
    }
}

/* Set to 1 to require only one finger to start the experiment (debug/measurement mode) */
#define DEBUG_SINGLE_FINGER 0

/* After this many ms of pulsing, hold outputs continuously ON to stabilise temperature */
#define STABILIZE_AFTER_MS 5000

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
static bool is_new_cmd = false;

/* ---------- Pulse task ---------- */
#define PULSE_NOTIFY_START 1
#define PULSE_NOTIFY_STOP  2

/* Number of pulse cycles over which the pause ramps up logarithmically 
Higher values make the pause ramp more gradually, but also mean it takes longer to reach the full pause duration.
Lower values make the pause ramp more quickly, but may cause a more abrupt change in pause duration between cycles.
*/
#define RAMP_CYCLES 8

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

        int64_t run_start_us = esp_timer_get_time();
        uint32_t cycle = 0;
        while (1) {
            /* ---- stabilise phase: once STABILIZE_AFTER_MS has elapsed, lock the
               ramp at full pause_ms so average power stays constant.
               Constant average power → thermal equilibrium (temperature plateaus). ---- */
            bool stabilised = (esp_timer_get_time() - run_start_us) >= (int64_t)STABILIZE_AFTER_MS * 1000;
            if (stabilised && cycle <= RAMP_CYCLES) {
                cycle = RAMP_CYCLES; /* skip remaining ramp steps */
                ESP_LOGI(TAG, "\033[33mSTABLE — fixed duty cycle, pause=%" PRIu32 "ms\033[0m", pause_ms);
            }
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

            /* Log ramp: pause grows from ~0 ms to pause_ms over RAMP_CYCLES cycles */
            uint32_t current_pause;
            if (cycle < RAMP_CYCLES && pause_ms > 0) {
                float t = log2f((float)(cycle + 1)) / log2f((float)(RAMP_CYCLES + 1));
                current_pause = (uint32_t)(pause_ms * t);
            } else {
                current_pause = pause_ms;
            }
            ESP_LOGI(TAG, "PAUSE %" PRIu32 "ms (ramp cycle %" PRIu32 ")", current_pause, cycle);
            cycle++;

            xTaskNotifyWait(0, ULONG_MAX, &note, pdMS_TO_TICKS(current_pause));
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

/* Touch hysteresis: activate above thresh_on (+12.5% above benchmark),
   deactivate below thresh_off (+4% above benchmark, i.e. between baseline and thresh_on) */

static touch_sensor_handle_t  s_sens_handle;
static touch_channel_handle_t s_chan_handle[TOUCH_CHANNEL_COUNT];
static uint32_t s_channel_thresh_on[TOUCH_CHANNEL_COUNT]  = {};
static uint32_t s_channel_thresh_off[TOUCH_CHANNEL_COUNT] = {};
static bool     s_ch_state[TOUCH_CHANNEL_COUNT]           = {};

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
        s_channel_thresh_on[i]  = cfg.active_thresh[0];           /* benchmark + 12.5% */
        s_channel_thresh_off[i] = benchmark[0] + (benchmark[0] / 25); /* benchmark +  4%  */
        ESP_LOGI(TAG, "CH%d benchmark=%" PRIu32 " thresh_on=%" PRIu32 " thresh_off=%" PRIu32,
                 touch_chan_ids[i], benchmark[0], s_channel_thresh_on[i], s_channel_thresh_off[i]);
    }
}

/* ---------- Main ---------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ntc_init();
    wifi_init();

    outputs_init();

    s_cmd_mutex = xSemaphoreCreateMutex();
    mqtt_init(&s_cmd, s_cmd_mutex, &is_new_cmd);

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
        float temps_c[NTC_COUNT];
        ntc_read_all(temps_c);
        ESP_LOGI(TAG, "NTC temp: GPIO1=%.1f °C  GPIO2=%.1f °C  GPIO3=%.1f °C",
                 temps_c[0], temps_c[1], temps_c[2]);

        uint32_t data[TOUCH_SAMPLE_CFG_NUM] = {};
        bool     ch_active[TOUCH_CHANNEL_COUNT];

        for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
            touch_channel_read_data(s_chan_handle[i], TOUCH_CHAN_DATA_TYPE_SMOOTH, data);
            if (!s_ch_state[i] && data[0] > s_channel_thresh_on[i])
                s_ch_state[i] = true;
            else if (s_ch_state[i] && data[0] < s_channel_thresh_off[i])
                s_ch_state[i] = false;
            ch_active[i] = s_ch_state[i];
            ESP_LOGI(TAG, "CH%d: %s value=%" PRIu32 " on=%" PRIu32 " off=%" PRIu32,
                     touch_chan_ids[i],
                     ch_active[i] ? "\033[31mON\033[0m" : "OFF",
                     data[0], s_channel_thresh_on[i], s_channel_thresh_off[i]);
        }

#if DEBUG_SINGLE_FINGER
        /* Debug mode: any single finger is enough to start */
        bool all_now = false;
        for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
            if (ch_active[i]) { all_now = true; break; }
        }
#else
        bool all_now = true;
        for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
            if (!ch_active[i]) { all_now = false; break; }
        }
#endif

        if (all_now && !all_active && is_new_cmd) {
            all_active  = true;
            touch_start = esp_timer_get_time();

            xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);
            memcpy(last_pulse_ms, s_cmd.pulse_ms, sizeof(s_cmd.pulse_ms));
            last_pause_ms = s_cmd.pause_ms;
            is_new_cmd = false;
            xSemaphoreGive(s_cmd_mutex);

            ESP_LOGI(TAG, "\033[32mALL ON — pulse_ms=[%" PRIu32 ",%" PRIu32 ",%" PRIu32 "] pause=%" PRIu32 "ms\033[0m",
                     last_pulse_ms[0], last_pulse_ms[1], last_pulse_ms[2], last_pause_ms);
            ESP_LOGI(TAG, "\033[32m---------Starting outputs...\033[0m");
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
            mqtt_publish_result(elapsed_ms, touched, last_pulse_ms, last_pause_ms, temps_c);
        }

        ESP_LOGI(TAG, "\033[32m-------------\033[0m");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
