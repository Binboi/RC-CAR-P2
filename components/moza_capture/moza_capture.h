#include "tusb.h"
#include "tusb_config.h"
#include "host/usbh.h"
#include "class/hid/hid_host.h"
#include "esp_private/usb_phy.h"

extern uint8_t moza_data[69];

void moza_capture_init(void);

void tuh_mount_cb(uint8_t daddr);
void cb_get();
void cb_ctrl();
void cb_set();



