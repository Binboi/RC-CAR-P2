#include "moza_parser.h"
#include <stdio.h>


// ─────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────

// Steering: -1.0 (full left) | 0.0 (center) | 1.0 (full right)
static inline float normalize_steering(uint16_t raw)
{
    if (raw >= MOZA_STEER_CENTER) {
        return (float)(raw - MOZA_STEER_CENTER) / (float)(MOZA_STEER_MAX - MOZA_STEER_CENTER);
    } else {
        return -((float)(MOZA_STEER_CENTER - raw) / (float)(MOZA_STEER_CENTER - MOZA_STEER_MIN));
    }
}

// Pedals: 0.0 (released) | 1.0 (fully pressed)
static inline float normalize_pedal(uint16_t raw)
{
    if (raw <= MOZA_PEDAL_RELEASED){
        
        return 0.0f;

    }

    if (raw >= MOZA_PEDAL_MAX){
        
        return 1.0f;

    }

    return (float)(raw - MOZA_PEDAL_RELEASED) / (float)(MOZA_PEDAL_MAX - MOZA_PEDAL_RELEASED);
}


//////////////////////
//Reading functions
//////////////////////

moza_input_t read_moza(const uint8_t report[], uint16_t len)
{
    moza_input_t input = {0};

    if (len < MOZA_REPORT_LEN)       return input;
    if (report[0] != MOZA_REPORT_ID) return input;

    // Read raw values (big-endian: high byte first)
    input.steering_raw = (report[MOZA_STEERING_OFFSET] << 8) | report[MOZA_STEERING_OFFSET + 1];
    input.throttle_raw = (report[MOZA_THROTTLE_OFFSET] << 8) | report[MOZA_THROTTLE_OFFSET + 1];
    input.brake_raw = (report[MOZA_BRAKE_OFFSET] << 8) | report[MOZA_BRAKE_OFFSET + 1];

    // Normalize to float
    input.steering = normalize_steering(input.steering_raw);
    input.throttle = normalize_pedal(input.throttle_raw);
    input.brake    = normalize_pedal(input.brake_raw);

    return input;
}

void moza_print(const moza_input_t input)
{
    printf("Steering: %.3f (raw: 0x%04X) | Throttle: %.3f (raw: 0x%04X) | Brake: %.3f (raw: 0x%04X)\n",
        input.steering, input.steering_raw,
        input.throttle, input.throttle_raw,
        input.brake,    input.brake_raw);
}