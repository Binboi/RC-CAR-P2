#include "moza_capture.h"

const tusb_rhport_init_t rh_init = {
    .role = TUSB_ROLE_HOST,
    .speed = TUSB_SPEED_FULL,
  };

uint8_t moza_data[69];

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
    (void) desc_report;
    (void) desc_len;
    printf("HID device mounted: dev_addr=%u instance=%u\n", dev_addr, instance);
    tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
    (void) dev_addr;
    (void) instance;
    printf("HID device unmounted: dev_addr=%u instance=%u\n", dev_addr, instance);
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {

    printf("Got report: len=%u\n", len);
    
    memcpy(moza_data, report, len);
    tuh_hid_receive_report(dev_addr, instance);

    printf("Steering: %u %u\n", moza_data[29], moza_data[28]);

}

void moza_capture_init(void){
    usb_phy_handle_t phy_hdl;
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .target = USB_PHY_TARGET_INT,
        .otg_mode = USB_OTG_MODE_HOST,
        .otg_speed = USB_PHY_SPEED_FULL,
        .ext_io_conf = NULL,
        .otg_io_conf = NULL,
    };
usb_new_phy(&phy_conf, &phy_hdl);
}

///HANDSHAKE STUFF

static uint8_t dev_addr;
static uint8_t buf[7];

//packet 41
void tuh_mount_cb(uint8_t daddr){
    dev_addr = daddr;
    static tusb_control_request_t setup = {
    .bmRequestType = 0xA1,
    .bRequest      = 0x21,
    .wValue        = 0,
    .wIndex        = 0,
    .wLength       = 7
  };

  tuh_xfer_t xfer = {
    .daddr       = dev_addr,
    .ep_addr     = 0,
    .setup       = &setup,
    .buffer      = buf,
    .complete_cb = cb_get
  };

  tuh_control_xfer(&xfer);
}

//packet 43
void cb_get(tuh_xfer_t* xfer){
    static tusb_control_request_t setup = {
        .bmRequestType = 0x21,
        .bRequest      = 0x22, // SET_CONTROL_LINE_STATE
        .wValue        = 0x0001, // DTR = 1 (important)
        .wIndex        = 0,
        .wLength       = 0
    };

    tuh_xfer_t xfer2 = {
        .daddr       = dev_addr,
        .ep_addr     = 0,
        .setup       = &setup,
        .buffer      = NULL,
        .complete_cb = cb_ctrl
    };

  tuh_control_xfer(&xfer2);
}

    //packet 45
    void cb_ctrl(tuh_xfer_t* xfer){
        static uint8_t coding[7] = {
            0x00, 0xC2, 0x01, 0x00, // 115200
            0x00,
            0x00,
            0x08
    };

    static tusb_control_request_t setup = {
        .bmRequestType = 0x21,
        .bRequest      = 0x20, // SET_LINE_CODING
        .wValue        = 0,
        .wIndex        = 0,
        .wLength       = 7
    };

    tuh_xfer_t xfer3 = {
        .daddr       = dev_addr,
        .ep_addr     = 0,
        .setup       = &setup,
        .buffer      = coding,
        .complete_cb = cb_set
    };

  tuh_control_xfer(&xfer3);
}

//packet 47
void cb_set(){
    static tusb_control_request_t setup = {
        .bmRequestType = 0xA1,
        .bRequest      = 0x21,
        .wValue        = 0,
        .wIndex        = 0,
        .wLength       = 7
    };

    tuh_xfer_t xfer4 = {
        .daddr       = dev_addr,
        .ep_addr     = 0,
        .setup       = &setup,
        .buffer      = buf,
        .complete_cb = NULL
    };

  tuh_control_xfer(&xfer4);
}
