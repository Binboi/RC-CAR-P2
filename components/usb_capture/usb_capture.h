#pragma once
#include <stdint.h>

#define MOZA_REPORT_LEN 69

//function to get packets
typedef void (*moza_report_cb_t)(const uint8_t *data, uint16_t len);


typedef struct {

    uint8_t report_type;
    uint16_t steering_raw; 
    uint16_t throttle_raw;
    uint16_t brake_raw;

} moza_report_t;

//Put the usb data into the moza_report_t struct
typedef void (*moza_report_cb_t)(const moza_report_t *report);

//Function to capture the moza data
void usb_start_capt(moza_report_cb_t);

//Function to stop moza data capture
void stop_stop_capt(void);
