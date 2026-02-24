#pragma once

//ESP32 library for error messages
#include "esp_err.h"

//force everything to be read in C code just to be safe
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and start the USB capture component.
 *        Starts two internal FreeRTOS tasks:
 *          - usb_daemon_task : handles USB host library events (device connect/disconnect)
 *          - usb_client_task : registers as a USB host client, opens device, reads raw HID reports
 *        Raw bytes received from the MOZA (or any HID device) are printed to the serial console.
 */
esp_err_t usb_capture_start(void);

/**
 * @brief Stop and clean up USB capture tasks and the USB host library.
 */
void usb_capture_stop(void);

#ifdef __cplusplus
}
#endif