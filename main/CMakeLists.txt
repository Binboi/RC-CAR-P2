idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES radio
                    REQUIRES usb_capture
                    PRIV_REQUIRES driver esp_adc
                    )


