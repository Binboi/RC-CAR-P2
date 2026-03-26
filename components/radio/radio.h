#pragma once

#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// NRF24 registers
    //power, transmit or receive mode, error checking mode on or off
#define NRF_CONFIG    0x00
    //A byte the NRF module sends back that informs of the status of each bit in the transmit byte
#define NRF_STATUS    0x07
    //picks the power of transmission and the data trasnfer rate
#define NRF_RF_SETUP  0x06
    //picks the radio frequency channel
#define NRF_RF_CH     0x05
    //sets RX address
#define NRF_RX_ADDR   0x0A
    //sets transmit address
#define NRF_TX_ADDR   0x10
    //sets how many bytes per packet to expect
#define NRF_RX_PW_P0  0x11

// SPI commands
    //read register with AN OR (keeps command bits 0x000 to define a read byte)
#define NRF_R_REGISTER  0x00
    //write register with AN OR (changes command bits to 0x001 to define a write byte)
#define NRF_W_REGISTER  0x20
    //send back or read the received packet
#define NRF_R_RX_PAYLOAD 0x61
    //writes data that the NRF module wil later transmit
#define NRF_W_TX_PAYLOAD 0xA0
    //clears the bytes stored in tx that one doesnt want to send anymore
#define NRF_FLUSH_TX     0xE1
    //clears the register which stores the read bytes from the received module
#define NRF_FLUSH_RX     0xE2
    //test byte to get information from a register
#define NRF_NOP 0xFF

//controller side radio pin defines
#define PIN_CSN   10
#define PIN_CE    9

#define PIN_MISO  13
#define PIN_MOSI  11
#define PIN_SCK   12

//how many bytes to expect per packet, steering, throttle, braking, drift yaw
#define PAYLOAD_SIZE  4

void csn_low();
void csn_high();
void ce_low();
void ce_high();

//configures the write register of the NRF
void nrf_write_reg(uint8_t reg, uint8_t val);

//reads the register of the NRF radio module
uint8_t nrf_read_reg(uint8_t reg);

// writes multiple bytes (for addresses)
void nrf_write_reg_multi(uint8_t reg, uint8_t *data, uint8_t len);

//configures the radio module
void nrf_init();



//radio transmit function
void nrf_transmit(uint8_t *data, uint8_t len);

//stars receiving mode on radio
void nrf_start_rx();

//stop receiveing mode
void nrf_stop_rx();

//flush the buffer in receiver
void nrf_flush_rx();

//clear buffer at tx
void nrf_flush_tx();

//flag to check if nrf has data stored in buffer
bool nrf_data_available();

//receive data and write into NRF module buffers for later extraction
void nrf_receive(uint8_t *buf, uint8_t len);