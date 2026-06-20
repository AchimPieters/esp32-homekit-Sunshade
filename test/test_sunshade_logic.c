/**
   Copyright 2026 Achim Pieters | StudioPieters®

   Host-side unit tests for the pure sunshade logic. These compile with a plain
   host compiler (no ESP-IDF) and run in CI to guard the hardware-independent
   behaviour: relay polarity, position math, sensor conversions and hysteresis.

   Build & run:
       cc -std=c11 -Wall -Wextra -Werror -I main test/test_sunshade_logic.c -o /tmp/t && /tmp/t
 **/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "sunshade_logic.h"

static int g_failures = 0;
static int g_checks   = 0;

#define CHECK(cond) do {                                                   \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            g_failures++;                                                   \
            printf("  FAIL: %s  (line %d)\n", #cond, __LINE__);            \
        }                                                                   \
    } while (0)

#define CHECK_FLOAT(a, b) do {                                             \
        g_checks++;                                                         \
        if (fabs((double)(a) - (double)(b)) > 1e-3) {                      \
            g_failures++;                                                   \
            printf("  FAIL: %s ~= %s  (%.5f vs %.5f, line %d)\n",          \
                   #a, #b, (double)(a), (double)(b), __LINE__);            \
        }                                                                   \
    } while (0)

static void test_relay_output_level(void) {
    printf("relay_output_level\n");
    // Active-high board: ON -> 1, OFF -> 0.
    CHECK(relay_output_level(1, true)  == 1);
    CHECK(relay_output_level(1, false) == 0);
    // Active-low board (common opto-isolated module): ON -> 0, OFF -> 1.
    CHECK(relay_output_level(0, true)  == 0);
    CHECK(relay_output_level(0, false) == 1);
}

static void test_clamp_position(void) {
    printf("clamp_position\n");
    CHECK(clamp_position(-5)  == 0);
    CHECK(clamp_position(0)   == 0);
    CHECK(clamp_position(42)  == 42);
    CHECK(clamp_position(100) == 100);
    CHECK(clamp_position(101) == 100);
}

static void test_position_delta(void) {
    printf("position_delta_per_tick\n");
    // 20 s full travel, 500 ms tick -> 2.5 % per tick.
    CHECK_FLOAT(position_delta_per_tick(20000, 500), 2.5f);
    // 10 s full travel, 500 ms tick -> 5 % per tick.
    CHECK_FLOAT(position_delta_per_tick(10000, 500), 5.0f);
    // Guard against divide-by-zero (treated as 1 ms travel).
    CHECK(position_delta_per_tick(0, 500) > 0.0f);
}

static void test_sensor_hysteresis(void) {
    printf("sensor_hysteresis\n");
    // Below close threshold while open -> no action.
    CHECK(sensor_hysteresis(false, 70, 80, 50) == SENSOR_NO_ACTION);
    // At/above close threshold while open -> close.
    CHECK(sensor_hysteresis(false, 80, 80, 50) == SENSOR_TRIGGER_CLOSE);
    CHECK(sensor_hysteresis(false, 95, 80, 50) == SENSOR_TRIGGER_CLOSE);
    // In the hysteresis gap while closed -> stay closed (no action).
    CHECK(sensor_hysteresis(true, 60, 80, 50) == SENSOR_NO_ACTION);
    // Below reopen threshold while closed -> reopen.
    CHECK(sensor_hysteresis(true, 49, 80, 50) == SENSOR_TRIGGER_REOPEN);
    // Exactly at reopen threshold while closed -> still closed (strict <).
    CHECK(sensor_hysteresis(true, 50, 80, 50) == SENSOR_NO_ACTION);
}

static void test_wind_speed_ds(void) {
    printf("wind_speed_ds\n");
    // Full scale 2270 mV maps to 140 dm/s (14.0 m/s).
    CHECK(wind_speed_ds(2270, 2270, 140) == 140);
    // Zero input -> zero speed.
    CHECK(wind_speed_ds(0, 2270, 140) == 0);
    // Half scale -> ~half speed.
    CHECK(wind_speed_ds(1135, 2270, 140) == 70);
    // Defensive: invalid full scale -> 0, never divides by zero.
    CHECK(wind_speed_ds(1000, 0, 140) == 0);
}

static void test_bh1750_lux(void) {
    printf("bh1750_raw_to_lux\n");
    CHECK(bh1750_raw_to_lux(0)     == 0);
    // Datasheet conversion: lux = raw / 1.2.
    CHECK(bh1750_raw_to_lux(12)    == 10);
    CHECK(bh1750_raw_to_lux(60000) == 50000);
}

static void test_sht3x(void) {
    printf("sht3x conversions + CRC\n");
    // Datasheet endpoints: raw 0 -> -45 C, raw 65535 -> +130 C.
    CHECK_FLOAT(sht3x_raw_to_celsius(0),     -45.0f);
    CHECK_FLOAT(sht3x_raw_to_celsius(65535), 130.0f);
    // Mid-scale ~ 42.5 C.
    CHECK_FLOAT(sht3x_raw_to_celsius(32768),  42.502f);
    // Humidity endpoints and clamp.
    CHECK_FLOAT(sht3x_raw_to_humidity(0),     0.0f);
    CHECK_FLOAT(sht3x_raw_to_humidity(65535), 100.0f);
    CHECK_FLOAT(sht3x_raw_to_humidity(32768), 50.0008f);
    // Sensirion datasheet CRC check value.
    CHECK(sensirion_crc8(0xBE, 0xEF) == 0x92);
    CHECK(sensirion_crc8(0x00, 0x00) == 0x81);
}

int main(void) {
    printf("== sunshade logic unit tests ==\n");
    test_relay_output_level();
    test_clamp_position();
    test_position_delta();
    test_sensor_hysteresis();
    test_wind_speed_ds();
    test_bh1750_lux();
    test_sht3x();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures != 0) {
        printf("RESULT: FAIL\n");
        return EXIT_FAILURE;
    }
    printf("RESULT: PASS\n");
    return EXIT_SUCCESS;
}
