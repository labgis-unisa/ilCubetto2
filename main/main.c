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
#define NTC_BETA         3940.0f    /* NTC B25/50 coefficient - TDK B57421V2103 (range 25–50°C matches operating point) */
#define ADC_MAX_VAL      4095.0f    /* 12-bit ADC */
#define NTC_OVERSAMPLE   16         /* ADC reads per median window */
#define NTC_MIN_VALID_C  15.0f      /* readings below this are treated as invalid (open circuit / noise) */
#define NTC_EMA_ALPHA    0.2f       /* EMA smoothing factor: lower = smoother but slower (0.1–0.3 typical) */

static const adc_channel_t s_ntc_channels[NTC_COUNT] = {
    ADC_CHANNEL_1,  /* GPIO2 POLLICE*/
    ADC_CHANNEL_2,  /* GPIO3 INDICE*/
    ADC_CHANNEL_0,  /* GPIO1 MEDIO*/
};

/* Per-channel calibration offsets (°C).
   s_ntc_offset_fixed: measured vs thermocouple reference (hardware trim).
   s_ntc_offset_dyn:   computed at startup so all channels read the same initial value. */
static const float s_ntc_offset_fixed[NTC_COUNT] = {
    20.1f,  /* GPIO2 */
    20.4f,  /* GPIO3 */
    20.1f,  /* GPIO1 */
};

/* Set to 1 to require only one finger to start the experiment (debug/measurement mode) */
#define DEBUG_SINGLE_FINGER 0

/* Set to 1: when all fingers are released, disable all channels and stop heating until
   a new MQTT command arrives. Set to 0: release has no effect on the heater state
   (heating continues until explicitly stopped via MQTT or a new touch session). */
#define STOP_ON_TOUCH_RELEASE 1

/* ---------- Temperature setpoint controller ---------- */
#define TEMP_SETPOINT_DEFAULT_C  43.0f  /* target skin-contact temperature, overridable via MQTT */
#define TEMP_HYST_C               0.5f  /* hysteresis band half-width */
#define TEMP_SAFETY_MARGIN_C      3.0f  /* NTC thermal lag compensation: heater turns OFF at (setpoint - margin)
                                           so real surface temperature stays at or below setpoint */
#define TEMP_POLL_MS             200    /* NTC polling period inside pulse_task bang-bang loop */
#define TEMP_NTC_SETTLE_MS        10    /* outputs OFF settling time before ADC read to avoid heater interference */

static float s_setpoint_c = TEMP_SETPOINT_DEFAULT_C; /* updated from MQTT */
static SemaphoreHandle_t s_setpoint_mutex;

static float s_ntc_offset_dyn[NTC_COUNT] = {0.0f, 0.0f, 0.0f};

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
    /* offset_c combines the fixed hardware trim and the dynamic inter-channel equalisation */
    return t_kelvin - 273.15f - offset_c;
}

static float s_ntc_ema[NTC_COUNT];        /* EMA state, initialised on first valid read */
static bool  s_ntc_ema_init[NTC_COUNT];   /* true once EMA has been seeded */

/* Insertion-sort median on a small array of ints */
static int _median(int *a, int n)
{
    for (int i = 1; i < n; i++) {
        int key = a[i], j = i - 1;
        while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
        a[j + 1] = key;
    }
    return a[n / 2];
}

static void ntc_read_all(float out_celsius[NTC_COUNT])
{
    for (int i = 0; i < NTC_COUNT; i++) {
        int raws[NTC_OVERSAMPLE];
        for (int s = 0; s < NTC_OVERSAMPLE; s++) {
            ESP_ERROR_CHECK(adc_oneshot_read(s_adc_handle, s_ntc_channels[i], &raws[s]));
        }
        /* Median removes impulse spikes that would corrupt a plain average */
        int med = _median(raws, NTC_OVERSAMPLE);
        float t = ntc_raw_to_celsius(med, s_ntc_offset_fixed[i] + s_ntc_offset_dyn[i]);

        if (t >= NTC_MIN_VALID_C) {
            if (!s_ntc_ema_init[i]) {
                s_ntc_ema[i]      = t;   /* seed EMA with first valid reading */
                s_ntc_ema_init[i] = true;
            } else {
                s_ntc_ema[i] = NTC_EMA_ALPHA * t + (1.0f - NTC_EMA_ALPHA) * s_ntc_ema[i];
            }
        } else {
            ESP_LOGW(TAG, "NTC[%d] median raw=%d → %.1f °C below minimum, using EMA %.1f °C",
                     i, med, t, s_ntc_ema[i]);
        }
        out_celsius[i] = s_ntc_ema[i];
    }
}

/* ---------- Output GPIOs ---------- */
static const gpio_num_t output_gpios[OUTPUT_COUNT] = {
    GPIO_NUM_7,
    GPIO_NUM_8,
    GPIO_NUM_9,
};

static void outputs_all_off(void)
{
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        gpio_set_level(output_gpios[i], 0);
    }
}

/* ---------- Pulse task — pure bang-bang closed loop ---------- */
#define PULSE_NOTIFY_START 1
#define PULSE_NOTIFY_STOP  2

static TaskHandle_t s_pulse_task;

/* Per-channel enable mask, updated from MQTT, protected by s_setpoint_mutex */
static bool s_channel_enabled[OUTPUT_COUNT] = {false, false, false};

/* Last channel mask received from a command message (never cleared at stop), protected by s_setpoint_mutex */
static bool s_last_cmd_channels[OUTPUT_COUNT] = {false, false, false};

/* Shared temperature stats written by pulse_task, read by app_main for MQTT publish */
static SemaphoreHandle_t s_temps_mutex;
static float             s_temps_snapshot[NTC_COUNT];
static float             s_temp_max[NTC_COUNT];
static float             s_temp_sum[NTC_COUNT];
static uint32_t          s_temp_count;

static void pulse_task(void *arg)
{
    while (1) {
        /* Wait for start notification */
        uint32_t note;
        do {
            xTaskNotifyWait(0, ULONG_MAX, &note, portMAX_DELAY);
        } while (note != PULSE_NOTIFY_START);

        bool ch_on[OUTPUT_COUNT] = {false};

        while (1) {
            /* Check for stop */
            if (xTaskNotifyWait(0, ULONG_MAX, &note, 0) == pdTRUE) {
                if (note == PULSE_NOTIFY_STOP) break;
            }

            /* Briefly cut all outputs before reading NTC to eliminate heater
               switching noise that corrupts the ADC when outputs are ON. */
            outputs_all_off();
            vTaskDelay(pdMS_TO_TICKS(TEMP_NTC_SETTLE_MS));

            float temps[NTC_COUNT];
            ntc_read_all(temps);

            /* Restore outputs that were ON before the measurement window */
            for (int i = 0; i < OUTPUT_COUNT; i++) {
                if (ch_on[i]) gpio_set_level(output_gpios[i], 1);
            }

            xSemaphoreTake(s_setpoint_mutex, portMAX_DELAY);
            float setpoint = s_setpoint_c;
            bool ch_enabled[OUTPUT_COUNT];
            memcpy(ch_enabled, s_channel_enabled, sizeof(ch_enabled));
            xSemaphoreGive(s_setpoint_mutex);

            ESP_LOGI(TAG, "NTC temp: %.1f °C  %.1f °C  %.1f °C  setpoint=%.1f °C",
                     temps[0], temps[1], temps[2], setpoint);

            /* Bang-bang with coldest-channel gating and safety margin:
               - OFF threshold: (setpoint - SAFETY_MARGIN_C) — heater stops early to
                 absorb thermal lag; real surface temperature reaches setpoint after
                 the NTC catches up, but never exceeds it.
               - ON threshold: (setpoint - SAFETY_MARGIN_C - HYST_C) — all channels
                 must be below this before any re-ignites (coldest-channel gating). */
            float off_thresh = setpoint - TEMP_SAFETY_MARGIN_C;
            float on_thresh  = off_thresh - TEMP_HYST_C;

            /* Disabled channels are forced off immediately */
            for (int i = 0; i < OUTPUT_COUNT; i++) {
                if (!ch_enabled[i] && ch_on[i]) {
                    ch_on[i] = false;
                    gpio_set_level(output_gpios[i], 0);
                    ESP_LOGI(TAG, "CTRL CH%d OFF (disabled)", i);
                }
            }

            /* Coldest-channel gating: consider only enabled channels */
            bool all_cold = true;
            for (int i = 0; i < OUTPUT_COUNT; i++) {
                if (ch_enabled[i] && temps[i] >= on_thresh) { all_cold = false; break; }
            }

            for (int i = 0; i < OUTPUT_COUNT; i++) {
                if (!ch_enabled[i]) continue;
                if (!ch_on[i] && all_cold) {
                    ch_on[i] = true;
                    gpio_set_level(output_gpios[i], 1);
                    ESP_LOGI(TAG, "CTRL CH%d ON  (%.1f °C, all < %.1f °C)", i, temps[i], on_thresh);
                } else if (ch_on[i] && temps[i] >= off_thresh) {
                    ch_on[i] = false;
                    gpio_set_level(output_gpios[i], 0);
                    ESP_LOGI(TAG, "CTRL CH%d OFF (%.1f °C >= %.1f °C)", i, temps[i], off_thresh);
                }
            }

            /* Update shared snapshot for app_main */
            xSemaphoreTake(s_temps_mutex, portMAX_DELAY);
            for (int i = 0; i < NTC_COUNT; i++) {
                s_temps_snapshot[i] = temps[i];
                if (temps[i] > s_temp_max[i]) s_temp_max[i] = temps[i];
                s_temp_sum[i] += temps[i];
            }
            s_temp_count++;
            xSemaphoreGive(s_temps_mutex);

            vTaskDelay(pdMS_TO_TICKS(TEMP_POLL_MS));
        }

        outputs_all_off();
        ESP_LOGI(TAG, "pulse_task stopped — all outputs OFF");
    }
}

static void outputs_init(void)
{
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        gpio_reset_pin(output_gpios[i]);
        gpio_set_direction(output_gpios[i], GPIO_MODE_OUTPUT);
        gpio_set_level(output_gpios[i], 0);
    }

    s_temps_mutex = xSemaphoreCreateMutex();
    xTaskCreate(pulse_task, "pulse", 4096, NULL, 5, &s_pulse_task);
}

static void outputs_start(void)
{
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
        /* Compute adaptive threshold from benchmark; CH5 (index 1) uses +8% for higher sensitivity */
        uint32_t thresh = (touch_chan_ids[i] == 5)
                          ? benchmark[0] + (benchmark[0] / 12)  /* +8% margin for CH5 */
                          : benchmark[0] + (benchmark[0] / 8);  /* +12.5% margin for other channels */
        if (thresh < 1000) thresh = 1000;
        if (thresh > 60000) thresh = 60000;
        touch_channel_config_t cfg = {
            .active_thresh    = {thresh},
            .charge_speed     = TOUCH_CHARGE_SPEED_7,
            .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        };
        ESP_ERROR_CHECK(touch_sensor_reconfig_channel(s_chan_handle[i], &cfg));
        s_channel_thresh_on[i]  = cfg.active_thresh[0];               /* benchmark + 12.5% */
        s_channel_thresh_off[i] = benchmark[0] + (benchmark[0] / 25); /* benchmark +  4%  */
        ESP_LOGI(TAG, "CH%d benchmark=%" PRIu32 " thresh_on=%" PRIu32 " thresh_off=%" PRIu32,
                 touch_chan_ids[i], benchmark[0], s_channel_thresh_on[i], s_channel_thresh_off[i]);
    }
}

/* ---------- NTC dynamic calibration ---------- */
/* Take NTC_CAL_SAMPLES averaged readings, then shift every channel by the
   same offset so they all report the mean of their initial temperatures.
   This removes inter-channel bias without altering the absolute scale
   (the fixed hardware offsets in s_ntc_offset_fixed still anchor accuracy). */
#define NTC_CAL_SAMPLES 32

static void ntc_calibrate(void)
{
    float sums[NTC_COUNT] = {0.0f};

    for (int s = 0; s < NTC_CAL_SAMPLES; s++) {
        float t[NTC_COUNT];
        ntc_read_all(t);
        for (int i = 0; i < NTC_COUNT; i++) sums[i] += t[i];
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    float means[NTC_COUNT];
    for (int i = 0; i < NTC_COUNT; i++) {
        means[i] = sums[i] / NTC_CAL_SAMPLES;
    }

    /* GPIO1 is the empirically calibrated reference channel (index 2 in s_ntc_channels).
       Align all other channels to its initial reading. */
    float ref_mean = means[2];

    for (int i = 0; i < NTC_COUNT; i++) {
        /* positive offset → subtract → lowers the reading of hotter channels */
        s_ntc_offset_dyn[i] = means[i] - ref_mean;
        ESP_LOGI(TAG, "NTC[%d] cal mean=%.2f °C  dyn_offset=%+.2f °C",
                 i, means[i], s_ntc_offset_dyn[i]);
    }
    ESP_LOGI(TAG, "NTC calibration done — target=%.2f °C (GPIO1 reference)", ref_mean);
}

/* ---------- Current command ---------- */
static SemaphoreHandle_t s_cmd_mutex;
static mqtt_cmd_t s_cmd = {
    .temp_setpoint_c  = TEMP_SETPOINT_DEFAULT_C,
    .channel_enabled  = {false, false, false},
};
static bool is_new_cmd = false;

/* ---------- Main ---------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ntc_init();
    ntc_calibrate();

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

    wifi_init();

    s_setpoint_mutex = xSemaphoreCreateMutex();
    outputs_init();

    s_cmd_mutex = xSemaphoreCreateMutex();
    mqtt_init(&s_cmd, s_cmd_mutex, &is_new_cmd);
    ESP_LOGI(TAG, "Touch sensor running");

    bool    all_active  = false;
    int64_t touch_start = 0;

    while (1) {
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

        /* Apply new setpoint and channel mask from MQTT — accepted while running or at start */
        if (is_new_cmd) {
            xSemaphoreTake(s_cmd_mutex, portMAX_DELAY);
            float new_sp = s_cmd.temp_setpoint_c;
            bool  new_en[OUTPUT_COUNT];
            memcpy(new_en, s_cmd.channel_enabled, sizeof(new_en));
            is_new_cmd = false;
            xSemaphoreGive(s_cmd_mutex);

            xSemaphoreTake(s_setpoint_mutex, portMAX_DELAY);
            s_setpoint_c = new_sp;
            memcpy(s_channel_enabled, new_en, sizeof(s_channel_enabled));
            memcpy(s_last_cmd_channels, new_en, sizeof(s_last_cmd_channels));
            xSemaphoreGive(s_setpoint_mutex);

            ESP_LOGI(TAG, "Setpoint=%.1f °C  channels=[%d,%d,%d]",
                     new_sp, new_en[0], new_en[1], new_en[2]);
        }

        if (all_now && !all_active && s_setpoint_c > 0) {
            all_active  = true;
            touch_start = esp_timer_get_time();

            xSemaphoreTake(s_temps_mutex, portMAX_DELAY);
            for (int i = 0; i < NTC_COUNT; i++) {
                s_temp_max[i] = s_temps_snapshot[i];
                s_temp_sum[i] = 0.0f;
            }
            s_temp_count = 0;
            xSemaphoreGive(s_temps_mutex);

            ESP_LOGI(TAG, "\033[32mALL ON — setpoint=%.1f °C\033[0m", s_setpoint_c);
            outputs_start();

        } else if (!all_now && all_active) {
#if STOP_ON_TOUCH_RELEASE
            all_active = false;
            outputs_stop();

            /* Reset channel mask so a new command is required before next session */
            xSemaphoreTake(s_setpoint_mutex, portMAX_DELAY);
            for (int i = 0; i < OUTPUT_COUNT; i++) s_channel_enabled[i] = false;
            xSemaphoreGive(s_setpoint_mutex);
#endif
            uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - touch_start) / 1000);
            ESP_LOGI(TAG, "\033[32mALL OFF — touch held %" PRIu32 " ms\033[0m", elapsed_ms);

            uint8_t touched[TOUCH_CHANNEL_COUNT];
            for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
                touched[i] = ch_active[i] ? 1 : 0;
            }

            float temp_max_snapshot[NTC_COUNT];
            float temp_avg_snapshot[NTC_COUNT];
            xSemaphoreTake(s_temps_mutex, portMAX_DELAY);
            memcpy(temp_max_snapshot, s_temp_max, sizeof(s_temp_max));
            for (int i = 0; i < NTC_COUNT; i++) {
                temp_avg_snapshot[i] = s_temp_count > 0
                    ? s_temp_sum[i] / (float)s_temp_count
                    : s_temp_max[i];
            }
            xSemaphoreGive(s_temps_mutex);

            xSemaphoreTake(s_setpoint_mutex, portMAX_DELAY);
            float sp = s_setpoint_c;
            bool ch_enabled_snapshot[OUTPUT_COUNT];
            memcpy(ch_enabled_snapshot, s_last_cmd_channels, sizeof(ch_enabled_snapshot));
            xSemaphoreGive(s_setpoint_mutex);

            mqtt_publish_result(elapsed_ms, touched, sp, ch_enabled_snapshot, temp_max_snapshot, temp_avg_snapshot);
        }

        ESP_LOGI(TAG, "\033[32m-------------\033[0m");
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
