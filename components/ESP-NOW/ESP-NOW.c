#include <stdio.h>
#include "ESP-NOW.h"

const uint8_t car_addr[6] = {0xF4, 0x65, 0x0B, 0xBB, 0x77, 0xF8};
size_t control_len = 6;
packet_t packet_drive_rcv;

void ESPNOWconfig(){

    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    //Set wifi parameters to default
    const wifi_init_config_t default_config = WIFI_INIT_CONFIG_DEFAULT();

    //create a report variable
    esp_err_t report;

    vTaskDelay(10/ portTICK_PERIOD_MS);
    //initialize wifi
    report = esp_wifi_init(&default_config);
        if (report == ESP_OK) { printf("WIFI Init Success.\n\r"); }
        else { printf("WIFI Init Failed: %d\n\r", report); }
    //set wifi mode to station (default)
    report = esp_wifi_set_mode(WIFI_MODE_STA);
        if (report == ESP_OK) { printf("WIFI Mode Success.\n\r"); }
        else { printf("WIFI Mode Failed: %d\n\r", report); }
    //start wifi
    report = esp_wifi_start();
        if (report == ESP_OK) { printf("WIFI Start Success.\n\r"); }
        else { printf("WIFI Start Failed: %d\n\r", report); }
    //Initialize espnow
    report = esp_now_init();
        if (report == ESP_OK) { printf("ESP-NOW Init Success.\n\r"); }
        else { printf("ESP-NOW Init Failed: %d\n\r", report); }

    //add peer (car)
    esp_now_peer_info_t car = {
        .peer_addr = {0xF4, 0x65, 0x0B, 0xBB, 0x77, 0xF8},
        .channel = 0,
        .ifidx = WIFI_IF_STA,
        .encrypt = 0,
    };
    //register peer (car)
    esp_now_add_peer(&car);
}