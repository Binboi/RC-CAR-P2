#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

// Lower number = higher priority
#define CFG_TUSB_RHPORT0_MODE    (OPT_MODE_HOST | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_OS              OPT_OS_FREERTOS

// CFG_TUD_HID is for gaming controllers/wheels
#define CFG_TUH_ENABLED          1
#define CFG_TUD_ENABLED          0
#define CFG_TUH_HID              1
#define CFG_TUD_HID              0
#define CFG_TUD_CDC              0
#define CFG_TUD_MSC              0
#define CFG_TUSB_OS              OPT_OS_FREERTOS
#define CFG_TUSB_MCU             OPT_MCU_ESP32S3
#define CFG_TUH_MAX_SPEED     OPT_MODE_FULL_SPEED
#define BOARD_TUH_RHPORT      0

// HID buffer size (set to 64 for high-res wheels)
#define CFG_TUD_HID_EP_BUFSIZE   69

#ifdef __cplusplus
 }
#endif

#endif