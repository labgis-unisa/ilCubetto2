#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

#define OUTPUT_PULSE_MS 300

static void outputs_init(void)
{
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        gpio_reset_pin(output_gpios[i]);
        gpio_set_direction(output_gpios[i], GPIO_MODE_OUTPUT);
        gpio_set_level(output_gpios[i], 0);
    }
}

static void outputs_pulse(const uint8_t mask[OUTPUT_COUNT], uint32_t duration_ms)
{
    if (duration_ms == 0) {
        duration_ms = OUTPUT_PULSE_MS;
    }
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        gpio_set_level(output_gpios[i], mask[i] ? 1 : 0);
    }
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    for (int i = 0; i < OUTPUT_COUNT; i++) {
        gpio_set_level(output_gpios[i], 0);
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
static volatile bool s_chan_active[TOUCH_CHANNEL_COUNT];
static volatile int  s_active_index = -1;

static bool touch_on_active_cb(touch_sensor_handle_t sens_handle,
                               const touch_active_event_data_t *event,
                               void *user_ctx)
{
    for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
        if (touch_chan_ids[i] == (int)event->chan_id) {
            s_chan_active[i] = true;
            s_active_index   = i;
            break;
        }
    }
    return false;
}

static bool touch_on_inactive_cb(touch_sensor_handle_t sens_handle,
                                 const touch_inactive_event_data_t *event,
                                 void *user_ctx)
{
    for (int i = 0; i < TOUCH_CHANNEL_COUNT; i++) {
        if (touch_chan_ids[i] == (int)event->chan_id) {
            s_chan_active[i] = false;
            break;
        }
    }
    return false;
}

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

    touch_event_callbacks_t callbacks = {
        .on_active   = touch_on_active_cb,
        .on_inactive = touch_on_inactive_cb,
    };
    ESP_ERROR_CHECK(touch_sensor_register_callbacks(s_sens_handle, &callbacks, NULL));

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
