/**
   Copyright 2026 Achim Pieters | StudioPieters®

   Pure, hardware-independent helper logic for the HomeKit Sunshade.

   This header intentionally has NO ESP-IDF or FreeRTOS dependencies so the
   exact same functions used by the firmware can be compiled and unit-tested on
   a host machine in CI (see test/test_sunshade_logic.c).
 **/

#pragma once

#include <stdint.h>
#include <stdbool.h>

// ── Relay level mapping ─────────────────────────────────────────────────────
// Translate a desired ON/OFF state into the GPIO level to write, honouring the
// board's active level. active_level = 1 for active-high relay boards, 0 for the
// common opto-isolated active-low boards (IN = LOW energises the relay).
static inline int relay_output_level(int active_level, bool on) {
    int high = (active_level != 0) ? 1 : 0;
    return on ? high : (high ^ 1);
}

// ── Position helpers ────────────────────────────────────────────────────────
// Clamp a position estimate to the valid HomeKit 0..100 % range.
static inline int clamp_position(int pos) {
    if (pos < 0)   return 0;
    if (pos > 100) return 100;
    return pos;
}

// Percentage of travel covered in one movement tick of tick_ms, given the full
// travel time travel_ms. Used by the time-based position estimator.
static inline float position_delta_per_tick(uint32_t travel_ms, uint32_t tick_ms) {
    if (travel_ms == 0) {
        travel_ms = 1;
    }
    return 100.0f * (float)tick_ms / (float)travel_ms;
}

// ── Generic high-value protection hysteresis ────────────────────────────────
// Shared by the wind and lux protection logic: a high measured value closes the
// sunshade, and it only reopens once the value drops back below a lower reopen
// threshold. The gap between the two thresholds prevents rapid cycling.
typedef enum {
    SENSOR_NO_ACTION = 0,
    SENSOR_TRIGGER_CLOSE,
    SENSOR_TRIGGER_REOPEN,
} sensor_action_t;

static inline sensor_action_t sensor_hysteresis(bool closed, int value,
                                                int close_threshold,
                                                int reopen_threshold) {
    if (!closed && value >= close_threshold) {
        return SENSOR_TRIGGER_CLOSE;
    }
    if (closed && value < reopen_threshold) {
        return SENSOR_TRIGGER_REOPEN;
    }
    return SENSOR_NO_ACTION;
}

// ── Sensor conversions ──────────────────────────────────────────────────────
// Convert divided ADC millivolts to wind speed in dm/s (1 dm/s = 0.1 m/s).
static inline int wind_speed_ds(int mv, int full_scale_mv, int max_speed_ds) {
    if (full_scale_mv <= 0) {
        return 0;
    }
    int ds = (int)((float)mv / (float)full_scale_mv * (float)max_speed_ds);
    return ds < 0 ? 0 : ds;
}

// Convert a raw BH1750 H-resolution sample to lux (datasheet: lux = raw / 1.2).
// Round to nearest rather than truncate: 1.2f is slightly above 1.2, so plain
// truncation would systematically under-report (e.g. 60000 -> 49999).
static inline int bh1750_raw_to_lux(uint16_t raw) {
    return (int)((float)raw / 1.2f + 0.5f);
}

// ── SHT3x temperature / humidity conversions ────────────────────────────────
// Sensirion SHT3x datasheet conversions for a raw 16-bit sample.
static inline float sht3x_raw_to_celsius(uint16_t raw) {
    return -45.0f + 175.0f * (float)raw / 65535.0f;
}

static inline float sht3x_raw_to_humidity(uint16_t raw) {
    float rh = 100.0f * (float)raw / 65535.0f;
    if (rh < 0.0f)   return 0.0f;
    if (rh > 100.0f) return 100.0f;
    return rh;
}

// Sensirion CRC-8 over a 16-bit word (polynomial 0x31, init 0xFF). Used to
// validate each SHT3x word; datasheet check value: CRC(0xBE, 0xEF) == 0x92.
static inline uint8_t sensirion_crc8(uint8_t msb, uint8_t lsb) {
    uint8_t data[2] = { msb, lsb };
    uint8_t crc = 0xFF;
    for (int i = 0; i < 2; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31)
                               : (uint8_t)(crc << 1);
        }
    }
    return crc;
}
