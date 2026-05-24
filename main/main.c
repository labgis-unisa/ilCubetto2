#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/touch_sens.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

#define TAG "TOUCH"

/* Output GPIOs */
#define OUTPUT_COUNT 3
static const gpio_num_t output_gpios[OUTPUT_COUNT] = {
    GPIO_NUM_7,
    GPIO_NUM_8,
    GPIO_NUM_9,
};

#define OUTPUT_PULSE_MS  300
#define OUTPUT_PAUSE_MS  500

typedef struct {
    uint8_t  mask[OUTPUT_COUNT];
    uint32_t duration_ms;
    bool     pending;
    bool     active;        /* true while pulse_task is driving the outputs */
} pulse_request_t;

static SemaphoreHandle_t   s_pulse_mutex;
static pulse_request_t     s_pulse_req;
static TaskHandle_t        s_pulse_task;

static void outputs_all_off(void)
{
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        gpio_set_level(output_gpios[i], 0);
    }
}

static void pulse_task(void *arg)
{
    while (1) {
        /* Wait until a request arrives */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        xSemaphoreTake(s_pulse_mutex, portMAX_DELAY);
        uint8_t  mask[OUTPUT_COUNT];
        uint32_t duration_ms;
        memcpy(mask, s_pulse_req.mask, OUTPUT_COUNT);
        duration_ms         = s_pulse_req.duration_ms;
        s_pulse_req.pending = false;
        s_pulse_req.active  = true;
        xSemaphoreGive(s_pulse_mutex);

        /* Activate outputs */
        ESP_LOGI(TAG, "OUT ON  [%d %d %d] dur=%" PRIu32 "ms",
                 mask[0], mask[1], mask[2], duration_ms);
        for (int i = 0; i < OUTPUT_COUNT; i++) {
            gpio_set_level(output_gpios[i], mask[i] ? 1 : 0);
        }

        /* Wait up to duration_ms; a new notification cancels the pulse early */
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(duration_ms));

        xSemaphoreTake(s_pulse_mutex, portMAX_DELAY);
        s_pulse_req.active = false;
        xSemaphoreGive(s_pulse_mutex);

        outputs_all_off();
        ESP_LOGI(TAG, "OUT OFF [0 0 0]%s", notified ? " (cancelled)" : "");

        if (notified) {
            /* New request arrived during pulse — re-queue it */
            xTaskNotifyGive(s_pulse_task);
        }

        /* Mandatory pause between activations */
        vTaskDelay(pdMS_TO_TICKS(OUTPUT_PAUSE_MS));
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

static void outputs_pulse(const uint8_t mask[OUTPUT_COUNT], uint32_t duration_ms)
{
    if (duration_ms == 0) {
        duration_ms = OUTPUT_PULSE_MS;
    }

    xSemaphoreTake(s_pulse_mutex, portMAX_DELAY);
    bool same_mask = s_pulse_req.active &&
                     memcmp(s_pulse_req.mask, mask, OUTPUT_COUNT) == 0;
    memcpy(s_pulse_req.mask, mask, OUTPUT_COUNT);
    s_pulse_req.duration_ms = duration_ms;
    s_pulse_req.pending     = true;
    xSemaphoreGive(s_pulse_mutex);

    /* Don't interrupt a running pulse if the mask hasn't changed */
    if (!same_mask) {
        xTaskNotifyGive(s_pulse_task);
    }
}

/* Touch channels 4, 5, 6 → GPIO 4, 5, 6 on ESP32-S3 */
#define TOUCH_CHANNEL_COUNT  3

static const int touch_chan_ids[TOUCH_CHANNEL_COUNT] = {4, 5, 6};

/* Ratio of benchmark used as active threshold */
#define THRESH_RATIO  0.015f

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
        ESP_ERROR_CHECK(touch_channel_read_data(s_chan_handle[i], TOUCH_CHAN_DATA_TYPE_BENCHMARK, benchmark));
        touch_channel_config_t cfg = {
            .active_thresh    = {48000},  // Set the active threshold to a value above the benchmark, which can be tuned according to the actual test result},
            .charge_speed     = TOUCH_CHARGE_SPEED_7,
            .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        };
        ESP_ERROR_CHECK(touch_sensor_reconfig_channel(s_chan_handle[i], &cfg));
        ESP_LOGI(TAG, "CH%d benchmark=%" PRIu32 " threshold=%" PRIu32,
                 touch_chan_ids[i], benchmark[0], cfg.active_thresh[0]);
    }
}

void app_main(void)
{
    outputs_init();

    touch_sensor_sample_config_t sample_cfg[TOUCH_SAMPLE_CFG_NUM] = {
        TOUCH_SENSOR_V2_DEFAULT_SAMPLE_CONFIG(500, TOUCH_VOLT_LIM_L_0V5, TOUCH_VOLT_LIM_H_2V2)
    };
    touch_sensor_config_t sens_cfg = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(TOUCH_SAMPLE_CFG_NUM, sample_cfg);
    ESP_ERROR_CHECK(touch_sensor_new_controller(&sens_cfg, &s_sens_handle));

    touch_channel_config_t chan_cfg = {
        .active_thresh    = {48000},
        .charge_speed     = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
    };
    for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
        ESP_ERROR_CHECK(touch_sensor_new_channel(s_sens_handle, touch_chan_ids[i], &chan_cfg, &s_chan_handle[i]));
    }

    touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    ESP_ERROR_CHECK(touch_sensor_config_filter(s_sens_handle, &filter_cfg));

    touch_initial_scanning();

    ESP_ERROR_CHECK(touch_sensor_enable(s_sens_handle));
    ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(s_sens_handle));

    ESP_LOGI(TAG, "Touch sensor running");
        uint8_t output_mask[OUTPUT_COUNT] = {0,0,0};
    
        while (1) {
        uint32_t data[TOUCH_SAMPLE_CFG_NUM] = {};
            for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
                touch_channel_read_data(s_chan_handle[i], TOUCH_CHAN_DATA_TYPE_SMOOTH, data);   
                if (data[0] > 48000) {
                    output_mask[i] = 1;
                    outputs_pulse(output_mask, 0);
                    ESP_LOGI(TAG, "CH%d: \033[31mON value=%" PRIu32, touch_chan_ids[i], data[0]);
                } else {
                    output_mask[i] = 0;
                    ESP_LOGI(TAG, "CH%d: \033[0mOFF value=%" PRIu32, touch_chan_ids[i], data[0]);
                }
    }
            ESP_LOGI(TAG, "\033[32m-------------");
            vTaskDelay(pdMS_TO_TICKS(200));
}   
}
