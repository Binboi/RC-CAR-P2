idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES radio usb_capture ESP-NOW moza_capture espressif__tinyusb usb
                    PRIV_REQUIRES driver esp_adc esp_wifi nvs_flash
                    )

idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)

target_include_directories(${tusb_lib} PUBLIC ".")

target_compile_definitions(${tusb_lib} PUBLIC CFG_TUSB_MCU=OPT_MCU_ESP32S3)


