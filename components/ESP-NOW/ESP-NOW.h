#pragma once

#include <stdio.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"

extern const uint8_t car_addr[6];
extern size_t control_len;

typedef struct {
    uint16_t str_dat;
    uint16_t thrt_dat;
} packet_t;

extern packet_t packet_drive_rcv;

void ESPNOWconfig();

