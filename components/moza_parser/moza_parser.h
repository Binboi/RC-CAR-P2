#ifndef MOZA_PARSER_H
#define MOZA_PARSER_H

#include <stdint.h>
#include <stdbool.h>

//  HID report was nalyzed through Wireshark
//  69 byte long report
//
//  Byte 0: When its 0x01, it means its an input report
//  Bytes 1–2: Steering  (big-endian) 0x0000 left | 0x8023 center | 0xFFFF right
//  Bytes 5–6: Throttle, 0x0080 is released and 0xFF7F full
//  Bytes 11–12: Brake, 0x0080 is released and 0xFF7F full

#define MOZA_REPORT_ID       0x01
#define MOZA_REPORT_LEN      69

#define MOZA_STEERING_OFFSET 1
#define MOZA_THROTTLE_OFFSET 5
#define MOZA_BRAKE_OFFSET    11

#define MOZA_STEER_MIN       0x0000
#define MOZA_STEER_MAX       0xFFFF
#define MOZA_STEER_CENTER    0x8023

#define MOZA_PEDAL_RELEASED  0x0080
#define MOZA_PEDAL_MAX       0xFF7F


//Structure for reading raw outputs and their respective mapped variables
typedef struct {
    uint16_t steering_raw;  //raw u16 int
    uint16_t throttle_raw;  //raw u16 int
    uint16_t brake_raw; //raw u 16 int

    float steering; // -1.0 = full left, 0.0 = middle, 1.0 = full right
    float throttle; //  0.0 = released, 1.0 = pressed
    float brake;    //  0.0 = released, 1.0 = pressed
} moza_input_t;


//FUNCTIONS

// Read and parse a raw HID report from the Moza R3.
// Returns a moza_input_t with all values set to 0 if the report is invalid.
moza_input_t read_moza(const uint8_t report[], uint16_t len);

// Print all current values over UART (for debugging)
void moza_print(const moza_input_t input);

#endif // MOZA_PARSER_H