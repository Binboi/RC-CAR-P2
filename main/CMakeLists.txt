idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES radio
                    REQUIRES usb_capture
                    REQUIRES ESP-NOW
                    PRIV_REQUIRES driver esp_adc esp_wifi
                    )


