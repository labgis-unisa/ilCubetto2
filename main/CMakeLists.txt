idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_wifi esp_event mqtt esp_timer json esp_driver_touch_sens esp_driver_gpio)
