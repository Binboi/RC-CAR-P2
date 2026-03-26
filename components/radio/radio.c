#include <stdio.h>
#include "radio.h"
#include "esp_rom_sys.h"

static uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

static spi_device_handle_t spi;

void csn_low()  { gpio_set_level(PIN_CSN, 0); }
void csn_high() { gpio_set_level(PIN_CSN, 1); }
void ce_low()   { gpio_set_level(PIN_CE, 0); }
void ce_high()  { gpio_set_level(PIN_CE, 1); }

//returns rx buffer data into variable
static uint16_t spi_transfer(uint8_t data) {
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &data;
    uint16_t rx = 0;
    t.rx_buffer = &rx;
    spi_device_transmit(spi, &t);
    return rx;
}

//sends two bytes to NRF which configruse the tx, rx, crc etc
void nrf_write_reg(uint8_t reg, uint8_t val) {
    csn_low();
    spi_transfer(NRF_W_REGISTER | reg);
    spi_transfer(val);
    csn_high();
}

//configures the read register of the NRF
uint8_t nrf_read_reg(uint8_t reg) {
    csn_low();
    spi_transfer(NRF_R_REGISTER | reg);
    uint8_t val = spi_transfer(NRF_NOP);
    csn_high();
    return val;
}

// write multiple bytes (for addresses)
void nrf_write_reg_multi(uint8_t reg, uint8_t *data, uint8_t len) {
    csn_low();
    spi_transfer(NRF_W_REGISTER | reg);
    for (int i = 0; i < len; i++) {
        spi_transfer(data[i]);
    }
    csn_high();
}

void nrf_init() {
    //setup CE and CSN as outputs
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_CE) | (1ULL << PIN_CSN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    ce_low();
    csn_high();

    //setup SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
            //ignore, not using
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
            //8 MHz SPI rate
        .clock_speed_hz = 1000000,
        .mode = 0,
            //manual CSN control
        .spics_io_num = -1,
            //one byte transfer at a time         
        .queue_size = 1,
    };
    //initializations after setup above
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);

    //power-up delay
    vTaskDelay(5 / portTICK_PERIOD_MS); 

    nrf_write_reg(0x01, 0x00);  //dsiable auto ACK pipe 0
    nrf_write_reg(0x02, 0x01);  //enable pipe 0
    nrf_write_reg(0x03, 0x03);  //5 byte address width
    nrf_write_reg(0x04, 0x00);  //auto retransmit off
    nrf_write_reg(NRF_CONFIG, 0x0E);  //power up, TX mode, CRC enabled
    vTaskDelay(5 / portTICK_PERIOD_MS); 
    nrf_write_reg(NRF_RF_CH, 0x7C);  //channel 76
    nrf_write_reg(NRF_RF_SETUP, 0x22);  //250 Kbps
    nrf_write_reg(NRF_RX_PW_P0, PAYLOAD_SIZE); //expect 2 bytes
    nrf_write_reg_multi(NRF_TX_ADDR, address, 5); //who to send to
    nrf_write_reg_multi(NRF_RX_ADDR, address, 5); //who to listen on
}

//radio transmit function
void nrf_transmit(uint8_t *data, uint8_t len) {
    // write payload into NRF24's TX buffer
    csn_low();
    spi_transfer(NRF_W_TX_PAYLOAD);
    for (int i = 0; i < len; i++) {
        spi_transfer(data[i]);
    }
    csn_high();

    // pulse CE to fire the transmission
    ce_high();
    esp_rom_delay_us(15);
    ce_low();
}

void nrf_start_rx() {
    nrf_write_reg(NRF_CONFIG, 0x0B);
    ce_high();
}

void nrf_stop_rx(){
    ce_low();
}

void nrf_flush_rx(){
    csn_low();
    spi_transfer(NRF_FLUSH_RX);
    csn_high();
}
void nrf_flush_tx(){
    csn_low();
    spi_transfer(NRF_FLUSH_TX);
    csn_high();
}

bool nrf_data_available() {
    uint8_t status = nrf_read_reg(NRF_STATUS);
    return (status & 0x40);
}

void nrf_receive(uint8_t *buf, uint8_t len) {
    csn_low();
    spi_transfer(NRF_R_RX_PAYLOAD);
    for (int i = 0; i < len; i++) {
        buf[i] = spi_transfer(NRF_NOP);
    }
    csn_high();
    nrf_write_reg(NRF_STATUS, 0x40);
}