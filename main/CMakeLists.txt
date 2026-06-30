set(srcs "esp_hid_device_main.c"
         "esp_hid_gap.c"
         "servo_control.c")
set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid
                       PRIV_REQUIRES nvs_flash esp_driver_ledc esp_driver_gpio esp_driver_rmt)
