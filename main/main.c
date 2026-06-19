/**
   Copyright 2026 Achim Pieters | StudioPieters®

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   for more information visit https://www.studiopieters.nl
 **/

/*
 * TTP223 SUNSHADE VERSION
 * ───────────────────────
 * This version uses 3 external TTP223 capacitive touch modules.
 *
 * Hardware:
 *   UP    TTP223 SIG -> GPIO32
 *   STOP  TTP223 SIG -> GPIO33
 *   DOWN  TTP223 SIG -> GPIO27
 *
 * Physical lifecycle button:
 *   GND -> button -> GPIO25
 *
 * TTP223 modules should be configured as:
 *   - momentary mode
 *   - active-high output
 *   - VCC = 3V3
 *   - GND = GND
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <nvs.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>

#ifdef CONFIG_WIND_SENSOR_ENABLE
#include <esp_adc/adc_oneshot.h>
#endif

// rain sensor uses only GPIO — no extra include needed

#ifdef CONFIG_LUX_SENSOR_ENABLE
#include <driver/i2c_master.h>
#endif

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include "esp32-lcm.h"
#include "sunshade_logic.h"
#include <button.h>

// ── GPIO pin assignments ───────────────────────────────────────────────────────
#define LED_GPIO            CONFIG_ESP_LED_GPIO
#define BUTTON_GPIO         CONFIG_ESP_BUTTON_GPIO
#define RELAY_OPEN_GPIO     CONFIG_ESP_RELAY_OPEN_GPIO
#define RELAY_CLOSE_GPIO    CONFIG_ESP_RELAY_CLOSE_GPIO
#define RELAY_ACTIVE_LEVEL  CONFIG_ESP_RELAY_ACTIVE_LEVEL
#define RELAY_REVERSE_DELAY_MS CONFIG_ESP_RELAY_REVERSE_DELAY_MS

#define TTP_UP_GPIO         CONFIG_ESP_TTP_UP_GPIO
#define TTP_STOP_GPIO       CONFIG_ESP_TTP_STOP_GPIO
#define TTP_DOWN_GPIO       CONFIG_ESP_TTP_DOWN_GPIO

#define TTP_ACTIVE_LEVEL    CONFIG_ESP_TTP_ACTIVE_LEVEL

// ── Timing ────────────────────────────────────────────────────────────────────
#define DEFAULT_TRAVEL_MS       CONFIG_SUNSHADE_FULL_TRAVEL_TIME_MS
#define POS_UPDATE_INTERVAL_MS  500

#define ENDSTOP_BUFFER_FACTOR   1.5f
#define CAL_OPEN_MAX_MS         120000u

#define TTP_POLL_MS             CONFIG_ESP_TTP_POLL_MS
#define TTP_DEBOUNCE_MS         CONFIG_ESP_TTP_DEBOUNCE_MS
#define CAL_HOLD_TRIGGER_MS     3000u

// ── NVS ───────────────────────────────────────────────────────────────────────
#define NVS_NS          "shade"
#define NVS_CAL_DONE    "cal_done"
#define NVS_CAL_MS      "cal_ms"
#define NVS_LAST_POS    "last_pos"

// ── Wind sensor (optional) ────────────────────────────────────────────────────
#ifdef CONFIG_WIND_SENSOR_ENABLE
#define WIND_ADC_GPIO            CONFIG_WIND_SENSOR_ADC_GPIO
#define WIND_ADC_FULL_SCALE_MV   CONFIG_WIND_SENSOR_ADC_FULL_SCALE_MV
#define WIND_MAX_SPEED_DS        CONFIG_WIND_SENSOR_MAX_SPEED_DS
#define WIND_CLOSE_THRESHOLD_DS  CONFIG_WIND_SENSOR_CLOSE_THRESHOLD_DS
#define WIND_REOPEN_THRESHOLD_DS CONFIG_WIND_SENSOR_REOPEN_THRESHOLD_DS
#define WIND_POLL_MS             CONFIG_WIND_SENSOR_POLL_MS
#endif

// ── Rain sensor (optional) ────────────────────────────────────────────────────
#ifdef CONFIG_RAIN_SENSOR_ENABLE
#define RAIN_GPIO          CONFIG_RAIN_SENSOR_GPIO
#define RAIN_ACTIVE_LEVEL  CONFIG_RAIN_SENSOR_ACTIVE_LEVEL
#define RAIN_POLL_MS       CONFIG_RAIN_SENSOR_POLL_MS
#define RAIN_DEBOUNCE_MS   CONFIG_RAIN_SENSOR_DEBOUNCE_MS
#endif

// ── Light sensor (optional) ───────────────────────────────────────────────────
#ifdef CONFIG_LUX_SENSOR_ENABLE
#define LUX_SDA_GPIO        CONFIG_LUX_SENSOR_SDA_GPIO
#define LUX_SCL_GPIO        CONFIG_LUX_SENSOR_SCL_GPIO
#define LUX_I2C_ADDR        CONFIG_LUX_SENSOR_I2C_ADDR
#define LUX_CLOSE_LUX       CONFIG_LUX_SENSOR_CLOSE_THRESHOLD_LUX
#define LUX_REOPEN_LUX      CONFIG_LUX_SENSOR_REOPEN_THRESHOLD_LUX
#define LUX_POLL_MS         CONFIG_LUX_SENSOR_POLL_MS
#endif

// ── Log tags ──────────────────────────────────────────────────────────────────
static const char *TAG        = "SUNSHADE";
static const char *TAG_TOUCH  = "TTP223";
static const char *TAG_BUTTON = "BUTTON";
static const char *TAG_RELAY  = "RELAY";
static const char *TAG_CAL    = "CAL";
static const char *TAG_NVS    = "NVS";
#ifdef CONFIG_WIND_SENSOR_ENABLE
static const char *TAG_WIND   = "WIND";
#endif
#ifdef CONFIG_RAIN_SENSOR_ENABLE
static const char *TAG_RAIN   = "RAIN";
#endif
#ifdef CONFIG_LUX_SENSOR_ENABLE
static const char *TAG_LUX    = "LUX";
#endif

// ── Motion state ──────────────────────────────────────────────────────────────
typedef enum {
    MOTION_CLOSING = 0,
    MOTION_OPENING = 1,
    MOTION_STOPPED = 2,
} motion_dir_t;

static volatile int          s_cur_pos      = 0;
static volatile int          s_tgt_pos      = 0;
static volatile motion_dir_t s_motion       = MOTION_STOPPED;
static volatile TaskHandle_t s_move_task    = NULL;
static volatile bool         s_task_creating = false;
static portMUX_TYPE          s_task_mux     = portMUX_INITIALIZER_UNLOCKED;

// ── Calibration state ─────────────────────────────────────────────────────────
typedef enum {
    CAL_IDLE    = 0,
    CAL_CLOSING,
    CAL_OPENING,
} cal_state_t;

static volatile cal_state_t s_cal_state   = CAL_IDLE;
static volatile bool        s_cal_done    = false;
static volatile uint32_t    s_cal_ms      = DEFAULT_TRAVEL_MS;
static volatile uint32_t    s_cal_t0_ms   = 0;
static volatile bool        s_is_homing   = false;
static volatile bool        s_cal_success = false;

// ── Wind protection state ─────────────────────────────────────────────────────
#ifdef CONFIG_WIND_SENSOR_ENABLE
static volatile bool s_wind_closed    = false;
static volatile int  s_wind_saved_pos = 0;
#endif

// ── Rain protection state ─────────────────────────────────────────────────────
#ifdef CONFIG_RAIN_SENSOR_ENABLE
static volatile bool s_rain_closed    = false;
static volatile int  s_rain_saved_pos = 0;
#endif

// ── Light protection state ────────────────────────────────────────────────────
#ifdef CONFIG_LUX_SENSOR_ENABLE
static volatile bool s_lux_closed    = false;
static volatile int  s_lux_saved_pos = 0;
#endif

// Last direction the relays were energised in, for reversal dead-time tracking.
static volatile motion_dir_t s_relay_dir = MOTION_STOPPED;

// ── Forward declarations ───────────────────────────────────────────────────────
static void sunshade_stop(void);
static void sunshade_move_to(int target);
static void relays_all_off(void);
static void homekit_notify_position(void);
static void calibration_confirm_open(void);
static void hold_position_setter(homekit_value_t value);

// ── Timing helper ─────────────────────────────────────────────────────────────
static inline uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// ── NVS helpers ───────────────────────────────────────────────────────────────
static void nvs_load_calibration(void) {
    nvs_handle_t h;

    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG_NVS, "No calibration found; using default %lu ms",
                 (unsigned long)DEFAULT_TRAVEL_MS);
        return;
    }

    uint8_t done = 0;
    if (nvs_get_u8(h, NVS_CAL_DONE, &done) != ESP_OK) {
        done = 0;
    }

    if (done) {
        uint32_t ms = DEFAULT_TRAVEL_MS;
        if (nvs_get_u32(h, NVS_CAL_MS, &ms) == ESP_OK && ms > 0) {
            s_cal_ms   = ms;
            s_cal_done = true;
            ESP_LOGI(TAG_NVS, "Calibration loaded: %lu ms", (unsigned long)ms);
        } else {
            ESP_LOGW(TAG_NVS, "Calibration flag set but value invalid; using default");
        }
    } else {
        ESP_LOGI(TAG_NVS, "Device not calibrated; using default %lu ms",
                 (unsigned long)DEFAULT_TRAVEL_MS);
    }

    nvs_close(h);
}

static void nvs_save_calibration(uint32_t travel_ms) {
    nvs_handle_t h;

    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG_NVS, "Cannot open NVS for calibration save");
        return;
    }

    esp_err_t err = nvs_set_u8(h, NVS_CAL_DONE, 1);
    if (err == ESP_OK) err = nvs_set_u32(h, NVS_CAL_MS, travel_ms);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGE(TAG_NVS, "Calibration save failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG_NVS, "Calibration saved: %lu ms", (unsigned long)travel_ms);
    }
}

static uint8_t nvs_load_last_position(void) {
    nvs_handle_t h;
    uint8_t pos = 0;

    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return 0;
    }

    if (nvs_get_u8(h, NVS_LAST_POS, &pos) != ESP_OK || pos > 100) {
        pos = 0;
    }
    nvs_close(h);

    ESP_LOGI(TAG_NVS, "Last saved position: %d%%", pos);
    return pos;
}

static void nvs_save_last_position(uint8_t pos) {
    if (pos > 100) {
        pos = 100;
    }

    nvs_handle_t h;

    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }

    esp_err_t err = nvs_set_u8(h, NVS_LAST_POS, pos);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGW(TAG_NVS, "Failed to save last position: %s", esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG_NVS, "Saved target position: %d%%", pos);
    }
}

// ── LED ───────────────────────────────────────────────────────────────────────
static void led_write(bool on) {
    gpio_set_level(LED_GPIO, on ? 1 : 0);
}

// ── Relay control ─────────────────────────────────────────────────────────────
// Honour the board's active level (CONFIG_ESP_RELAY_ACTIVE_LEVEL) and enforce a
// dead time when reversing direction so an AC tubular motor is never switched
// straight from one direction to the other.
static void relays_all_off(void) {
    gpio_set_level(RELAY_OPEN_GPIO,  relay_output_level(RELAY_ACTIVE_LEVEL, false));
    gpio_set_level(RELAY_CLOSE_GPIO, relay_output_level(RELAY_ACTIVE_LEVEL, false));
    s_relay_dir = MOTION_STOPPED;
    ESP_LOGD(TAG_RELAY, "Both relays OFF");
}

static void relay_reverse_guard(motion_dir_t new_dir) {
    if (RELAY_REVERSE_DELAY_MS > 0 &&
        s_relay_dir != MOTION_STOPPED && s_relay_dir != new_dir) {
        gpio_set_level(RELAY_OPEN_GPIO,  relay_output_level(RELAY_ACTIVE_LEVEL, false));
        gpio_set_level(RELAY_CLOSE_GPIO, relay_output_level(RELAY_ACTIVE_LEVEL, false));
        ESP_LOGD(TAG_RELAY, "Reversal dead time %d ms", RELAY_REVERSE_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(RELAY_REVERSE_DELAY_MS));
    }
}

static void relay_activate_open(void) {
    relay_reverse_guard(MOTION_OPENING);
    gpio_set_level(RELAY_CLOSE_GPIO, relay_output_level(RELAY_ACTIVE_LEVEL, false));
    gpio_set_level(RELAY_OPEN_GPIO,  relay_output_level(RELAY_ACTIVE_LEVEL, true));
    s_relay_dir = MOTION_OPENING;
    ESP_LOGD(TAG_RELAY, "OPEN relay ON");
}

static void relay_activate_close(void) {
    relay_reverse_guard(MOTION_CLOSING);
    gpio_set_level(RELAY_OPEN_GPIO,  relay_output_level(RELAY_ACTIVE_LEVEL, false));
    gpio_set_level(RELAY_CLOSE_GPIO, relay_output_level(RELAY_ACTIVE_LEVEL, true));
    s_relay_dir = MOTION_CLOSING;
    ESP_LOGD(TAG_RELAY, "CLOSE relay ON");
}

// ── HomeKit characteristics ───────────────────────────────────────────────────
static void target_position_setter(homekit_value_t value);

static homekit_characteristic_t current_pos_ch =
    HOMEKIT_CHARACTERISTIC_(CURRENT_POSITION, 0);

static homekit_characteristic_t target_pos_ch =
    HOMEKIT_CHARACTERISTIC_(TARGET_POSITION, 0,
        .setter = target_position_setter);

static homekit_characteristic_t pos_state_ch =
    HOMEKIT_CHARACTERISTIC_(POSITION_STATE, 2);

static homekit_characteristic_t hold_pos_ch =
    HOMEKIT_CHARACTERISTIC_(HOLD_POSITION, false,
        .setter = hold_position_setter);

static void homekit_notify_position(void) {
    current_pos_ch.value = HOMEKIT_UINT8((uint8_t)s_cur_pos);
    target_pos_ch.value  = HOMEKIT_UINT8((uint8_t)s_tgt_pos);
    pos_state_ch.value   = HOMEKIT_UINT8((uint8_t)s_motion);

    homekit_characteristic_notify(&current_pos_ch, current_pos_ch.value);
    homekit_characteristic_notify(&target_pos_ch, target_pos_ch.value);
    homekit_characteristic_notify(&pos_state_ch, pos_state_ch.value);

    ESP_LOGD(TAG, "HomeKit notify: current=%d target=%d state=%d",
             s_cur_pos, s_tgt_pos, s_motion);
}

// ── Movement task ─────────────────────────────────────────────────────────────
static void movement_task(void *arg) {
    const uint32_t travel_ms = (s_cal_ms > 0) ? s_cal_ms : DEFAULT_TRAVEL_MS;
    const float    delta     = position_delta_per_tick(travel_ms, POS_UPDATE_INTERVAL_MS);

    motion_dir_t last_dir         = s_motion;
    float        pos_f            = (float)s_cur_pos;
    int          last_notified    = s_cur_pos;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POS_UPDATE_INTERVAL_MS));

        motion_dir_t dir = s_motion;

        if (dir != last_dir) {
            pos_f    = (float)s_cur_pos;
            last_dir = dir;
        }

        if (dir == MOTION_STOPPED) {
            relays_all_off();
            homekit_notify_position();
            break;
        }

        int tgt = s_tgt_pos;

        if (dir == MOTION_OPENING) {
            pos_f += delta;
            if (pos_f >= (float)tgt) {
                s_cur_pos = tgt;
                s_motion  = MOTION_STOPPED;
                relays_all_off();
                homekit_notify_position();
                ESP_LOGI(TAG, "Reached %d%% (open)", tgt);
                break;
            }
        } else {
            pos_f -= delta;
            if (pos_f <= (float)tgt) {
                s_cur_pos = tgt;
                s_motion  = MOTION_STOPPED;
                relays_all_off();
                homekit_notify_position();
                ESP_LOGI(TAG, "Reached %d%% (close)", tgt);
                break;
            }
        }

        if (pos_f < 0.0f) {
            pos_f = 0.0f;
        } else if (pos_f > 100.0f) {
            pos_f = 100.0f;
        }

        int new_pos = clamp_position((int)pos_f);
        s_cur_pos = new_pos;
        if (new_pos != last_notified) {
            last_notified = new_pos;
            homekit_notify_position();
        }
    }

    // Dual-core: critical section ensures the other core sees NULL immediately.
    taskENTER_CRITICAL(&s_task_mux);
    s_move_task = NULL;
    taskEXIT_CRITICAL(&s_task_mux);

    vTaskDelete(NULL);
}

static void movement_task_start(void) {
    // Two-phase flag prevents dual-core TOCTOU: both cores seeing NULL simultaneously
    // and each spawning a duplicate movement task.
    bool create = false;

    taskENTER_CRITICAL(&s_task_mux);
    if (s_move_task == NULL && !s_task_creating) {
        s_task_creating = true;
        create = true;
    }
    taskEXIT_CRITICAL(&s_task_mux);

    if (!create) {
        return;
    }

    if (xTaskCreate(movement_task, "sunshade_move", 4096, NULL, 5,
                    (TaskHandle_t *)&s_move_task) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create movement task");
        relays_all_off();
        s_motion = MOTION_STOPPED;
        homekit_notify_position();
    }

    taskENTER_CRITICAL(&s_task_mux);
    s_task_creating = false;
    taskEXIT_CRITICAL(&s_task_mux);
}

// ── Sunshade control ──────────────────────────────────────────────────────────
static bool is_locked(void) {
    if (s_cal_state != CAL_IDLE) {
        ESP_LOGW(TAG, "Command ignored: calibration in progress");
        return true;
    }

    if (s_is_homing) {
        ESP_LOGW(TAG, "Command ignored: homing in progress");
        return true;
    }

    return false;
}

static void sunshade_open(void) {
    if (is_locked()) {
        return;
    }

    if (s_cur_pos >= 100) {
        ESP_LOGI(TAG, "Already fully open");
        return;
    }

    ESP_LOGI(TAG, "Opening: %d%% -> 100%%", s_cur_pos);

    relays_all_off();
    s_tgt_pos           = 100;
    s_motion            = MOTION_OPENING;
    target_pos_ch.value = HOMEKIT_UINT8(100);

    relay_activate_open();
    homekit_notify_position();
    nvs_save_last_position(100);
    movement_task_start();
}

static void sunshade_close(void) {
    if (is_locked()) {
        return;
    }

    if (s_cur_pos <= 0) {
        ESP_LOGI(TAG, "Already fully closed");
        return;
    }

    ESP_LOGI(TAG, "Closing: %d%% -> 0%%", s_cur_pos);

    relays_all_off();
    s_tgt_pos           = 0;
    s_motion            = MOTION_CLOSING;
    target_pos_ch.value = HOMEKIT_UINT8(0);

    relay_activate_close();
    homekit_notify_position();
    nvs_save_last_position(0);
    movement_task_start();
}

static void sunshade_stop(void) {
    if (s_cal_state == CAL_OPENING) {
        calibration_confirm_open();
        return;
    }

    if (is_locked()) {
        return;
    }

    ESP_LOGI(TAG, "Stop at %d%%", s_cur_pos);

    s_motion            = MOTION_STOPPED;
    s_tgt_pos           = s_cur_pos;
    target_pos_ch.value = HOMEKIT_UINT8((uint8_t)s_cur_pos);

    relays_all_off();
    homekit_notify_position();
    nvs_save_last_position((uint8_t)s_cur_pos);
}

static void sunshade_move_to(int target) {
    if (is_locked()) {
        return;
    }

    if (target < 0) {
        target = 0;
    } else if (target > 100) {
        target = 100;
    }

    if (target == s_cur_pos) {
        sunshade_stop();
        return;
    }

    relays_all_off();

    s_tgt_pos           = target;
    target_pos_ch.value = HOMEKIT_UINT8((uint8_t)target);

    if (target > s_cur_pos) {
        ESP_LOGI(TAG, "Move to %d%% (opening)", target);
        s_motion = MOTION_OPENING;
        relay_activate_open();
    } else {
        ESP_LOGI(TAG, "Move to %d%% (closing)", target);
        s_motion = MOTION_CLOSING;
        relay_activate_close();
    }

    homekit_notify_position();
    nvs_save_last_position((uint8_t)target);
    movement_task_start();
}

// ── HomeKit setters ───────────────────────────────────────────────────────────
static void target_position_setter(homekit_value_t value) {
    if (value.format != homekit_format_uint8) {
        ESP_LOGE(TAG, "target_position: unexpected format %d", value.format);
        return;
    }

    ESP_LOGI(TAG, "HomeKit -> target_position: %d%%", value.uint8_value);
    sunshade_move_to((int)value.uint8_value);
}

static void hold_position_setter(homekit_value_t value) {
    if (value.format != homekit_format_bool) {
        ESP_LOGE(TAG, "hold_position: unexpected format %d", value.format);
        return;
    }

    if (value.bool_value) {
        ESP_LOGI(TAG, "HomeKit -> hold_position: stop");
        sunshade_stop();
    }
}

// ── Calibration ───────────────────────────────────────────────────────────────
static void calibration_led_task(void *arg) {
    while (s_cal_state != CAL_IDLE) {
        uint32_t half = (s_cal_state == CAL_CLOSING) ? 100 : 400;

        led_write(true);
        vTaskDelay(pdMS_TO_TICKS(half));
        led_write(false);
        vTaskDelay(pdMS_TO_TICKS(half));
    }

    if (s_cal_success) {
        // 5 short flashes = success
        for (int i = 0; i < 5; i++) {
            led_write(true);
            vTaskDelay(pdMS_TO_TICKS(60));
            led_write(false);
            vTaskDelay(pdMS_TO_TICKS(60));
        }
    } else {
        // 3 long flashes = failed / aborted / timeout
        for (int i = 0; i < 3; i++) {
            led_write(true);
            vTaskDelay(pdMS_TO_TICKS(400));
            led_write(false);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    vTaskDelete(NULL);
}

static void calibration_confirm_open(void) {
    uint32_t elapsed = now_ms() - s_cal_t0_ms;

    relays_all_off();

    if (elapsed < 2000) {
        ESP_LOGW(TAG_CAL, "Aborted: STOP pressed too soon (%lu ms)",
                 (unsigned long)elapsed);
        s_cal_success        = false;
        s_cal_state          = CAL_IDLE;
        s_cur_pos            = 0;
        s_tgt_pos            = 0;
        s_motion             = MOTION_STOPPED;
        current_pos_ch.value = HOMEKIT_UINT8(0);
        target_pos_ch.value  = HOMEKIT_UINT8(0);
        homekit_notify_position();
        return;
    }

    s_cal_ms             = elapsed;
    s_cal_done           = true;
    s_cal_success        = true;
    s_cur_pos            = 100;
    s_tgt_pos            = 100;
    s_motion             = MOTION_STOPPED;

    current_pos_ch.value = HOMEKIT_UINT8(100);
    target_pos_ch.value  = HOMEKIT_UINT8(100);
    s_cal_state          = CAL_IDLE;

    nvs_save_calibration(elapsed);
    nvs_save_last_position(100);
    homekit_notify_position();

    ESP_LOGI(TAG_CAL, "Calibration complete: travel time = %lu ms",
             (unsigned long)elapsed);
}

static void calibration_task(void *arg) {
    ESP_LOGI(TAG_CAL, "=== CALIBRATION START ===");

    s_cal_success = false;
    s_motion      = MOTION_STOPPED;
    relays_all_off();
    vTaskDelay(pdMS_TO_TICKS(300));

    // ── Phase 1: drive to closed end-stop ─────────────────────────────────────
    ESP_LOGI(TAG_CAL, "Phase 1: closing fully (LED: fast blink)...");
    s_cal_state = CAL_CLOSING;

    uint32_t close_ms = (uint32_t)(s_cal_ms * ENDSTOP_BUFFER_FACTOR);
    if (close_ms < 8000) {
        close_ms = 8000;
    }

    relay_activate_close();
    vTaskDelay(pdMS_TO_TICKS(close_ms));
    relays_all_off();

    s_cur_pos            = 0;
    s_tgt_pos            = 0;
    s_motion             = MOTION_STOPPED;
    current_pos_ch.value = HOMEKIT_UINT8(0);
    target_pos_ch.value  = HOMEKIT_UINT8(0);
    homekit_notify_position();

    vTaskDelay(pdMS_TO_TICKS(500));

    // ── Phase 2: open and wait for user to press STOP ─────────────────────────
    ESP_LOGI(TAG_CAL, "Phase 2: opening (LED: slow blink) - press STOP when fully open");

    s_cal_t0_ms          = now_ms();
    s_cal_state          = CAL_OPENING;
    s_motion             = MOTION_OPENING;
    target_pos_ch.value  = HOMEKIT_UINT8(100);
    relay_activate_open();
    homekit_notify_position();

    uint32_t t_start        = now_ms();
    uint32_t last_notify_ms = t_start;
    uint32_t last_log_ms    = t_start;
    uint32_t ref_ms         = (s_cal_ms > 0) ? s_cal_ms : DEFAULT_TRAVEL_MS;
    int      last_notified  = 0;

    while (s_cal_state == CAL_OPENING) {
        uint32_t now = now_ms();

        if ((now - t_start) > CAL_OPEN_MAX_MS) {
            ESP_LOGW(TAG_CAL, "Timeout: calibration aborted after %lu s",
                     (unsigned long)(CAL_OPEN_MAX_MS / 1000));
            relays_all_off();
            s_cal_success = false;
            s_cal_state   = CAL_IDLE;
            s_cur_pos     = 0;
            s_tgt_pos     = 0;
            s_motion      = MOTION_STOPPED;
            homekit_notify_position();
            break;
        }

        // Live position estimate: only notify HomeKit when integer value changes.
        if ((now - last_notify_ms) >= POS_UPDATE_INTERVAL_MS) {
            uint32_t elapsed = now - s_cal_t0_ms;
            int estimated    = (int)((float)elapsed / (float)ref_ms * 100.0f);
            if (estimated > 99) estimated = 99;
            s_cur_pos = estimated;
            if (estimated != last_notified) {
                last_notified = estimated;
                homekit_notify_position();
            }
            last_notify_ms = now;
        }

        // Serial progress log every second for installer/monitor feedback.
        if ((now - last_log_ms) >= 1000) {
            uint32_t elapsed = now - s_cal_t0_ms;
            int pct = (int)((float)elapsed / (float)ref_ms * 100.0f);
            if (pct > 99) pct = 99;
            ESP_LOGI(TAG_CAL, "Opening... ~%d%% (%lu s) - press STOP when fully open",
                     pct, (unsigned long)(elapsed / 1000));
            last_log_ms = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG_CAL, "=== CALIBRATION END (state: %s) ===",
             s_cal_success ? "SUCCESS" : "ABORTED");

    vTaskDelete(NULL);
}

static void calibration_start(void) {
    if (s_cal_state != CAL_IDLE) {
        return;
    }

    if (s_is_homing) {
        ESP_LOGW(TAG_CAL, "Cannot calibrate while homing");
        return;
    }

    if (xTaskCreate(calibration_task, "cal_task", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG_CAL, "Failed to create calibration task");
        return;
    }
    if (xTaskCreate(calibration_led_task, "cal_led", 1024, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG_CAL, "Failed to create calibration LED task");
    }
}

// ── Boot homing ───────────────────────────────────────────────────────────────
static void homing_task(void *arg) {
    uint8_t last_pos = (uint8_t)(uintptr_t)arg;

    ESP_LOGI(TAG, "Homing: closing fully to establish position 0%%...");

    s_is_homing = true;
    s_motion    = MOTION_CLOSING;

    uint32_t close_ms = (uint32_t)(s_cal_ms * ENDSTOP_BUFFER_FACTOR);
    if (close_ms < 8000) {
        close_ms = 8000;
    }

    relay_activate_close();
    vTaskDelay(pdMS_TO_TICKS(close_ms));
    relays_all_off();

    s_cur_pos = 0;
    s_tgt_pos = 0;
    s_motion  = MOTION_STOPPED;

    current_pos_ch.value = HOMEKIT_UINT8(0);
    target_pos_ch.value  = HOMEKIT_UINT8(0);
    homekit_notify_position();

    ESP_LOGI(TAG, "Homing: position 0%% established");

    vTaskDelay(pdMS_TO_TICKS(500));

    s_is_homing = false;

    if (last_pos > 0 && last_pos <= 100) {
        ESP_LOGI(TAG, "Homing: restoring to last target %d%%", last_pos);
        sunshade_move_to((int)last_pos);
    } else {
        ESP_LOGI(TAG, "Homing complete; sunshade at 0%%");
    }

    vTaskDelete(NULL);
}

// ── Identify ──────────────────────────────────────────────────────────────────
static volatile TaskHandle_t s_identify_task = NULL;

static void accessory_identify_task(void *args) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            led_write(true);
            vTaskDelay(pdMS_TO_TICKS(100));
            led_write(false);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    led_write(false);
    s_identify_task = NULL;
    vTaskDelete(NULL);
}

static void accessory_identify(homekit_value_t _value) {
    ESP_LOGI("INFORMATION", "Accessory identify");
    if (s_identify_task != NULL) {
        return;
    }
    xTaskCreate(accessory_identify_task, "identify",
                1024, NULL, 2, (TaskHandle_t *)&s_identify_task);
}

// ── HomeKit accessory definition ──────────────────────────────────────────────
#define DEVICE_NAME         "HomeKit Sunshade"
#define DEVICE_MANUFACTURER "StudioPieters\xc2\xae"
#define DEVICE_SERIAL       "NLDA4SQN1466"
#define DEVICE_MODEL        "SD466NL/A"

static homekit_characteristic_t name =
    HOMEKIT_CHARACTERISTIC_(NAME, DEVICE_NAME);
static homekit_characteristic_t manufacturer =
    HOMEKIT_CHARACTERISTIC_(MANUFACTURER, DEVICE_MANUFACTURER);
static homekit_characteristic_t serial =
    HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, DEVICE_SERIAL);
static homekit_characteristic_t model =
    HOMEKIT_CHARACTERISTIC_(MODEL, DEVICE_MODEL);
static homekit_characteristic_t revision =
    HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION, LIFECYCLE_DEFAULT_FW_VERSION);
static homekit_characteristic_t ota_trigger = API_OTA_TRIGGER;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"
homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id = 1,
                      .category = homekit_accessory_category_window_covering,
                      .services = (homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics = (homekit_characteristic_t*[]) {
            &name,
            &manufacturer,
            &serial,
            &model,
            &revision,
            HOMEKIT_CHARACTERISTIC(IDENTIFY, accessory_identify),
            NULL
        }),
        HOMEKIT_SERVICE(WINDOW_COVERING, .primary = true, .characteristics = (homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Sun Screen"),
            &current_pos_ch,
            &target_pos_ch,
            &pos_state_ch,
            &hold_pos_ch,
            &ota_trigger,
            NULL
        }),
        NULL
    }),
    NULL
};
#pragma GCC diagnostic pop

static homekit_server_config_t hk_config = {
    .accessories = accessories,
    .password    = CONFIG_ESP_SETUP_CODE,
    .setupId     = CONFIG_ESP_SETUP_ID,
};

// ── TTP223 digital touch inputs ───────────────────────────────────────────────
typedef struct {
    gpio_num_t gpio;
    bool stable;
    bool last_raw;
    uint32_t changed_at_ms;
} debounced_input_t;

static bool ttp_read_raw(gpio_num_t gpio) {
    return gpio_get_level(gpio) == TTP_ACTIVE_LEVEL;
}

static bool debounce_update(debounced_input_t *input) {
    bool raw     = ttp_read_raw(input->gpio);
    uint32_t now = now_ms();

    if (raw != input->last_raw) {
        input->last_raw      = raw;
        input->changed_at_ms = now;
    }

    if ((now - input->changed_at_ms) >= TTP_DEBOUNCE_MS) {
        input->stable = raw;
    }

    return input->stable;
}

static void ttp_gpio_init_one(gpio_num_t gpio) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void ttp_init(void) {
    ttp_gpio_init_one((gpio_num_t)TTP_UP_GPIO);
    ttp_gpio_init_one((gpio_num_t)TTP_STOP_GPIO);
    ttp_gpio_init_one((gpio_num_t)TTP_DOWN_GPIO);

    ESP_LOGI(TAG_TOUCH,
             "TTP223 inputs: UP=GPIO%d STOP=GPIO%d DOWN=GPIO%d active_level=%d debounce=%dms poll=%dms",
             TTP_UP_GPIO, TTP_STOP_GPIO, TTP_DOWN_GPIO,
             TTP_ACTIVE_LEVEL, TTP_DEBOUNCE_MS, TTP_POLL_MS);
}

static void ttp_task(void *arg) {
    // Seed from actual GPIO state: a sensor already active at boot must not
    // trigger a false rising edge on the first poll.
    uint32_t t0       = now_ms();
    bool init_up      = ttp_read_raw((gpio_num_t)TTP_UP_GPIO);
    bool init_stop    = ttp_read_raw((gpio_num_t)TTP_STOP_GPIO);
    bool init_down    = ttp_read_raw((gpio_num_t)TTP_DOWN_GPIO);

    debounced_input_t up = {
        .gpio          = (gpio_num_t)TTP_UP_GPIO,
        .stable        = init_up,
        .last_raw      = init_up,
        .changed_at_ms = t0,
    };

    debounced_input_t stop_in = {
        .gpio          = (gpio_num_t)TTP_STOP_GPIO,
        .stable        = init_stop,
        .last_raw      = init_stop,
        .changed_at_ms = t0,
    };

    debounced_input_t down = {
        .gpio          = (gpio_num_t)TTP_DOWN_GPIO,
        .stable        = init_down,
        .last_raw      = init_down,
        .changed_at_ms = t0,
    };

    bool up_prev   = init_up;
    bool stop_prev = init_stop;
    bool down_prev = init_down;

    // Wall-clock timestamp for hold detection avoids poll-jitter accumulation error.
    uint32_t stop_press_start = 0;
    bool     cal_armed        = false;

    for (;;) {
        bool up_now   = debounce_update(&up);
        bool stop_now = debounce_update(&stop_in);
        bool down_now = debounce_update(&down);

        if (up_now && !up_prev) {
            ESP_LOGI(TAG_TOUCH, "UP touched -> opening");
            sunshade_open();
        }

        if (down_now && !down_prev) {
            ESP_LOGI(TAG_TOUCH, "DOWN touched -> closing");
            sunshade_close();
        }

        if (stop_now) {
            if (!stop_prev) {
                stop_press_start = now_ms();
            }

            if (!cal_armed && (now_ms() - stop_press_start) >= CAL_HOLD_TRIGGER_MS) {
                ESP_LOGI(TAG_TOUCH, "STOP held %lu ms -> calibration trigger",
                         (unsigned long)(now_ms() - stop_press_start));
                cal_armed = true;
                calibration_start();
            }
        } else {
            if (stop_prev && !cal_armed) {
                if (s_cal_state == CAL_OPENING) {
                    ESP_LOGI(TAG_TOUCH, "STOP -> confirm fully open");
                    calibration_confirm_open();
                } else {
                    ESP_LOGI(TAG_TOUCH, "STOP touched -> stopping");
                    sunshade_stop();
                }
            }

            cal_armed = false;
        }

        up_prev   = up_now;
        stop_prev = stop_now;
        down_prev = down_now;

        vTaskDelay(pdMS_TO_TICKS(TTP_POLL_MS));
    }
}

// ── Physical button ───────────────────────────────────────────────────────────
static void button_callback(button_event_t event, void *context) {
    switch (event) {
    case button_event_single_press:
        ESP_LOGI(TAG_BUTTON, "Single press -> update/reboot");
        relays_all_off();
        lifecycle_request_update_and_reboot();
        break;

    case button_event_double_press:
        ESP_LOGI(TAG_BUTTON, "Double press -> HomeKit reset + restart");
        relays_all_off();
        homekit_server_reset();
        esp_restart();
        break;

    case button_event_long_press:
        ESP_LOGI(TAG_BUTTON, "Long press -> factory reset + reboot");
        relays_all_off();
        lifecycle_factory_reset_and_reboot();
        break;

    default:
        ESP_LOGI(TAG_BUTTON, "Unknown button event: %d", event);
        break;
    }
}

// ── GPIO init ─────────────────────────────────────────────────────────────────
static void gpio_init_all(void) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    led_write(false);

    gpio_reset_pin(RELAY_OPEN_GPIO);
    gpio_set_direction(RELAY_OPEN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_OPEN_GPIO, relay_output_level(RELAY_ACTIVE_LEVEL, false));

    gpio_reset_pin(RELAY_CLOSE_GPIO);
    gpio_set_direction(RELAY_CLOSE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_CLOSE_GPIO, relay_output_level(RELAY_ACTIVE_LEVEL, false));
    s_relay_dir = MOTION_STOPPED;

    ESP_LOGI(TAG, "GPIO: LED=%d RELAY_OPEN=%d RELAY_CLOSE=%d active_level=%d reverse_delay=%dms",
             LED_GPIO, RELAY_OPEN_GPIO, RELAY_CLOSE_GPIO,
             RELAY_ACTIVE_LEVEL, RELAY_REVERSE_DELAY_MS);
}

// ── WiFi ready ────────────────────────────────────────────────────────────────
static void on_wifi_ready(void) {
    static volatile bool homekit_started = false;

    if (homekit_started) {
        ESP_LOGI("INFORMATION", "HomeKit already running; skipping re-init");
        return;
    }

    ESP_LOGI("INFORMATION", "Starting HomeKit server...");
    homekit_server_init(&hk_config);
    homekit_started = true;
}

// ── Wind speed sensor ─────────────────────────────────────────────────────────
#ifdef CONFIG_WIND_SENSOR_ENABLE
/*
 * HWFS-1 anemometer — implementation notes (based on field research):
 *
 * POWER:     Passive / self-generating. NO external VCC required.
 *            The rotating cups drive a small DC generator in the base.
 *            Wiring: signal wire (black+stripe) and ground wire (solid black) only.
 *
 * OUTPUT:    0–3.3 V DC analog (HWFS-1 3.3 V version). Max output = 14 m/s.
 *            Although 3.3 V matches ESP32 Vcc, the ADC_ATTEN_DB_11 accurate range
 *            is only 150–2450 mV. Use R1=10 kΩ + R2=22 kΩ to bring 3.3 V down
 *            to 2.27 V, safely within the accurate range.
 *
 * FORMULA:   3.3 V = 14 m/s (manufacturer stated, linear).
 *            The HWFS-1 uses a DC generator; output is nonlinear at very low RPM
 *            (cogging torque, brush friction). Calibrate against a reference meter
 *            and adjust WIND_SENSOR_MAX_SPEED_DS if readings are off.
 *            Default Kconfig WIND_SENSOR_MAX_SPEED_DS = 140 (14.0 m/s).
 *
 * NOISE:     DC commutator generates ripple voltage. Cogging creates step-wise
 *            output near startup threshold. Hardware: place 10 kΩ + 10 µF RC
 *            low-pass filter (τ ≈ 100 ms) on the signal before the ESP32 ADC.
 *            Software: 16-sample average per reading (implemented below).
 *
 * ADC:       ESP32 internal ADC has ±10% nonlinearity without calibration.
 *            ADC_ATTEN_DB_11 useful range: 150 mV–2450 mV. Above this the
 *            reading saturates nonlinearly. Stay within range with the divider.
 *            ESP32 ADC reads unreliably below ~100 mV; the 30 mV resting offset
 *            is masked by the WIND_ADC_ZERO_OFFSET_MV threshold below.
 */

// Number of ADC samples averaged per reading to reduce commutator ripple.
#define WIND_ADC_SAMPLES  16

// ADC_ATTEN_DB_11 full-scale approximation (mV). Actual varies ±5% per chip.
#define WIND_ADC_ATTEN_FS_MV  2450

// Sensor resting offset at the ADC pin with R1=10k/R2=22k divider:
// ~33 mV sensor × 22/32 = ~23 mV. Use 30 mV threshold for safety margin.
#define WIND_ADC_ZERO_OFFSET_MV  30

static void wind_task(void *arg) {
    adc_unit_t    adc_unit;
    adc_channel_t adc_channel;

    if (adc_oneshot_io_to_channel(WIND_ADC_GPIO, &adc_unit, &adc_channel) != ESP_OK) {
        ESP_LOGE(TAG_WIND, "GPIO%d is not a valid ADC pin; wind task disabled", WIND_ADC_GPIO);
        vTaskDelete(NULL);
        return;
    }

    // ADC2 conflicts with WiFi on ESP32; only ADC1 (GPIO32-39) is safe.
    if (adc_unit != ADC_UNIT_1) {
        ESP_LOGE(TAG_WIND, "GPIO%d maps to ADC2 (conflicts with WiFi); use GPIO32-39",
                 WIND_ADC_GPIO);
        vTaskDelete(NULL);
        return;
    }

    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    // ADC_ATTEN_DB_11: accurate range 150–2450 mV. Use R1=10k/R2=22k divider
    // so 3.3 V sensor max maps to 2270 mV, safely within the accurate range.
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, adc_channel, &chan_cfg));

    ESP_LOGI(TAG_WIND,
             "HWFS-1 ready: GPIO%d | cal factor V×%d.%d m/s | "
             "close >= %d.%d m/s | reopen < %d.%d m/s | poll %lu ms | %d samples/reading",
             WIND_ADC_GPIO,
             (WIND_MAX_SPEED_DS * 1000 / WIND_ADC_FULL_SCALE_MV) / 10,
             (WIND_MAX_SPEED_DS * 1000 / WIND_ADC_FULL_SCALE_MV) % 10,
             WIND_CLOSE_THRESHOLD_DS / 10,  WIND_CLOSE_THRESHOLD_DS % 10,
             WIND_REOPEN_THRESHOLD_DS / 10, WIND_REOPEN_THRESHOLD_DS % 10,
             (unsigned long)WIND_POLL_MS,
             WIND_ADC_SAMPLES);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(WIND_POLL_MS));

        // Average WIND_ADC_SAMPLES readings to suppress commutator ripple and
        // brush-contact noise. Spread samples evenly within the poll interval.
        int32_t raw_sum = 0;
        int     valid   = 0;
        for (int s = 0; s < WIND_ADC_SAMPLES; s++) {
            int raw = 0;
            if (adc_oneshot_read(adc_handle, adc_channel, &raw) == ESP_OK) {
                raw_sum += raw;
                valid++;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        if (valid == 0) {
            ESP_LOGW(TAG_WIND, "All ADC reads failed");
            continue;
        }

        int raw_avg = (int)(raw_sum / valid);

        // Convert averaged raw counts to mV using the ADC_ATTEN_DB_11 full-scale.
        int mv = (int)((float)raw_avg * (float)WIND_ADC_ATTEN_FS_MV / 4095.0f);

        // Mask the resting offset (~33 mV sensor → ~20 mV at ADC after divider).
        // Also masks the ESP32 ADC's unreliable sub-100 mV region.
        if (mv <= WIND_ADC_ZERO_OFFSET_MV) {
            mv = 0;
        }

        // Linear conversion: speed_ds = mv × MAX_SPEED_DS / ADC_FULL_SCALE_MV
        // This implements: speed(m/s) = V_sensor × calibration_factor
        // where calibration_factor is derived from MAX_SPEED_DS and the divider ratio.
        int speed_ds = wind_speed_ds(mv, WIND_ADC_FULL_SCALE_MV, WIND_MAX_SPEED_DS);

        // Log at INFO level so installers can read raw mV for field calibration.
        ESP_LOGI(TAG_WIND, "%d.%d m/s (avg_raw=%d, %d mV)",
                 speed_ds / 10, speed_ds % 10, raw_avg, mv);

        sensor_action_t act = sensor_hysteresis(s_wind_closed, speed_ds,
                                                WIND_CLOSE_THRESHOLD_DS,
                                                WIND_REOPEN_THRESHOLD_DS);
        if (act == SENSOR_TRIGGER_CLOSE) {
            ESP_LOGW(TAG_WIND, "Wind %d.%d m/s >= %d.%d m/s: closing for protection",
                     speed_ds / 10, speed_ds % 10,
                     WIND_CLOSE_THRESHOLD_DS / 10, WIND_CLOSE_THRESHOLD_DS % 10);
            s_wind_saved_pos = s_tgt_pos;
            s_wind_closed    = true;
            sunshade_close();
        } else if (act == SENSOR_TRIGGER_REOPEN) {
            ESP_LOGI(TAG_WIND, "Wind %d.%d m/s < %d.%d m/s: restoring to %d%%",
                     speed_ds / 10, speed_ds % 10,
                     WIND_REOPEN_THRESHOLD_DS / 10, WIND_REOPEN_THRESHOLD_DS % 10,
                     s_wind_saved_pos);
            s_wind_closed = false;
            sunshade_move_to(s_wind_saved_pos);
        }
    }
}
#endif

// ── Rain sensor ───────────────────────────────────────────────────────────────
#ifdef CONFIG_RAIN_SENSOR_ENABLE
/*
 * MH-RD rain sensor — implementation notes:
 *
 * POWER:   VCC = 3.3 V, GND = GND. The module has an onboard LM393 comparator.
 *          Sensitivity is set via the onboard potentiometer.
 *
 * OUTPUT:  DO = digital output. Active-low by default (DO goes LOW when rain
 *          is detected). Configure RAIN_SENSOR_ACTIVE_LEVEL = 0 (default).
 *          AO (analog output) is not used.
 *
 * NOISE:   The sensing pad corrodes outdoors over time. Mount the pad under a
 *          sheltered overhang to limit exposure. The 2 s debounce prevents
 *          false triggers from brief condensation or dust.
 */
static void rain_task(void *arg) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RAIN_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = (RAIN_ACTIVE_LEVEL == 0) ? GPIO_PULLUP_ENABLE  : GPIO_PULLUP_DISABLE,
        .pull_down_en = (RAIN_ACTIVE_LEVEL == 1) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_LOGI(TAG_RAIN,
             "MH-RD ready: GPIO%d active_%s poll=%lums debounce=%lums",
             RAIN_GPIO,
             RAIN_ACTIVE_LEVEL ? "high" : "low",
             (unsigned long)RAIN_POLL_MS,
             (unsigned long)RAIN_DEBOUNCE_MS);

    bool     last_raw   = (gpio_get_level(RAIN_GPIO) == RAIN_ACTIVE_LEVEL);
    uint32_t changed_at = now_ms();
    bool     stable     = last_raw;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(RAIN_POLL_MS));

        bool     raw  = (gpio_get_level(RAIN_GPIO) == RAIN_ACTIVE_LEVEL);
        uint32_t now  = now_ms();

        if (raw != last_raw) {
            last_raw   = raw;
            changed_at = now;
        }

        if ((now - changed_at) >= RAIN_DEBOUNCE_MS) {
            bool new_stable = raw;

            if (new_stable && !stable) {
                ESP_LOGW(TAG_RAIN, "Rain detected: closing for protection (saved pos=%d%%)",
                         s_tgt_pos);
                s_rain_saved_pos = s_tgt_pos;
                s_rain_closed    = true;
                sunshade_close();
            } else if (!new_stable && stable) {
                ESP_LOGI(TAG_RAIN, "Rain stopped: restoring to %d%%", s_rain_saved_pos);
                s_rain_closed = false;
                sunshade_move_to(s_rain_saved_pos);
            }

            stable = new_stable;
        }
    }
}
#endif

// ── Ambient light sensor ──────────────────────────────────────────────────────
#ifdef CONFIG_LUX_SENSOR_ENABLE
/*
 * BH1750 (GY-302) ambient light sensor — implementation notes:
 *
 * BUS:     I2C. VCC = 3.3 V, GND = GND, SDA/SCL on the configured GPIOs.
 *          GY-302 breakouts carry onboard pull-ups; internal pull-ups are also
 *          enabled here as a safety net for bare modules.
 *
 * ADDRESS: 0x23 when ADDR is tied LOW (default), 0x5C when ADDR is tied HIGH.
 *
 * MODE:    Continuous high-resolution mode (1 lux resolution, ~120 ms/sample).
 *          Reading returns a big-endian 16-bit count; lux = count / 1.2.
 *
 * USE:     Bright sun closes the sunshade for shade; once the light drops below
 *          the reopen threshold the previous position is restored. The same
 *          high-value hysteresis as the wind protection is used.
 */
#define BH1750_CMD_POWER_ON      0x01
#define BH1750_CMD_CONT_HRES     0x10

static void lux_task(void *arg) {
    i2c_master_bus_config_t bus_cfg = {
        .clk_source                 = I2C_CLK_SRC_DEFAULT,
        .i2c_port                   = I2C_NUM_0,
        .scl_io_num                 = LUX_SCL_GPIO,
        .sda_io_num                 = LUX_SDA_GPIO,
        .glitch_ignore_cnt          = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus = NULL;
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) {
        ESP_LOGE(TAG_LUX, "I2C bus init failed (SDA=%d SCL=%d); lux task disabled",
                 LUX_SDA_GPIO, LUX_SCL_GPIO);
        vTaskDelete(NULL);
        return;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = LUX_I2C_ADDR,
        .scl_speed_hz    = 100000,
    };

    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK) {
        ESP_LOGE(TAG_LUX, "BH1750 not added at 0x%02X; lux task disabled", LUX_I2C_ADDR);
        i2c_del_master_bus(bus);
        vTaskDelete(NULL);
        return;
    }

    uint8_t cmd = BH1750_CMD_POWER_ON;
    bool ok = (i2c_master_transmit(dev, &cmd, 1, 1000) == ESP_OK);
    cmd = BH1750_CMD_CONT_HRES;
    ok = ok && (i2c_master_transmit(dev, &cmd, 1, 1000) == ESP_OK);
    if (!ok) {
        ESP_LOGE(TAG_LUX, "BH1750 not responding at 0x%02X; check wiring/address",
                 LUX_I2C_ADDR);
        i2c_master_bus_rm_device(dev);
        i2c_del_master_bus(bus);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG_LUX,
             "BH1750 ready: addr 0x%02X SDA=%d SCL=%d | close >= %d lux | "
             "reopen < %d lux | poll %d ms",
             LUX_I2C_ADDR, LUX_SDA_GPIO, LUX_SCL_GPIO,
             LUX_CLOSE_LUX, LUX_REOPEN_LUX, LUX_POLL_MS);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(LUX_POLL_MS));

        uint8_t raw[2] = {0};
        if (i2c_master_receive(dev, raw, sizeof(raw), 1000) != ESP_OK) {
            ESP_LOGW(TAG_LUX, "BH1750 read failed");
            continue;
        }

        uint16_t count = (uint16_t)((raw[0] << 8) | raw[1]);
        int lux = bh1750_raw_to_lux(count);

        ESP_LOGI(TAG_LUX, "%d lux (raw=%u)", lux, count);

        sensor_action_t act = sensor_hysteresis(s_lux_closed, lux,
                                                LUX_CLOSE_LUX, LUX_REOPEN_LUX);
        if (act == SENSOR_TRIGGER_CLOSE) {
            ESP_LOGW(TAG_LUX, "Light %d lux >= %d lux: closing for sun protection",
                     lux, LUX_CLOSE_LUX);
            s_lux_saved_pos = s_tgt_pos;
            s_lux_closed    = true;
            sunshade_close();
        } else if (act == SENSOR_TRIGGER_REOPEN) {
            ESP_LOGI(TAG_LUX, "Light %d lux < %d lux: restoring to %d%%",
                     lux, LUX_REOPEN_LUX, s_lux_saved_pos);
            s_lux_closed = false;
            sunshade_move_to(s_lux_saved_pos);
        }
    }
}
#endif

// ── Entry point ───────────────────────────────────────────────────────────────
void app_main(void) {
    ESP_ERROR_CHECK(lifecycle_nvs_init());
    lifecycle_log_post_reset_state("INFORMATION");
    ESP_ERROR_CHECK(lifecycle_configure_homekit(&revision, &ota_trigger, "INFORMATION"));

    gpio_init_all();

    nvs_load_calibration();

    ttp_init();

    if (xTaskCreate(ttp_task, "ttp_task", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TTP task");
    }

    // Physical lifecycle button.
    // Wiring: GND -> button -> GPIO25.
    // Active-low requires a pull-up. Configure it explicitly so the input
    // cannot float if the button library does not enable the pull-up itself.
    gpio_config_t btn_io = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn_io));

    ESP_LOGI(TAG_BUTTON, "Physical button GPIO%d level at boot: %d",
             BUTTON_GPIO, gpio_get_level(BUTTON_GPIO));

    button_config_t btn_cfg    = button_config_default(button_active_low);
    btn_cfg.max_repeat_presses = 3;
    btn_cfg.long_press_time    = 3000;

    if (button_create(BUTTON_GPIO, btn_cfg, button_callback, NULL)) {
        ESP_LOGE(TAG_BUTTON, "Failed to init button on GPIO%d", BUTTON_GPIO);
    }

    if (s_cal_done) {
        uint8_t last_pos = nvs_load_last_position();
        ESP_LOGI(TAG, "Calibrated: starting homing sequence (last pos: %d%%)", last_pos);
        if (xTaskCreate(homing_task, "homing", 4096,
                        (void *)(uintptr_t)last_pos, 4, NULL) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create homing task");
        }
    } else {
        ESP_LOGW(TAG, "Not calibrated: skipping homing.");
        ESP_LOGW(TAG, "Hold STOP touch pad for 3 s to start calibration.");
    }

#ifdef CONFIG_WIND_SENSOR_ENABLE
    if (xTaskCreate(wind_task, "wind_task", 3072, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create wind task");
    }
#endif

#ifdef CONFIG_RAIN_SENSOR_ENABLE
    if (xTaskCreate(rain_task, "rain_task", 2048, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create rain task");
    }
#endif

#ifdef CONFIG_LUX_SENSOR_ENABLE
    if (xTaskCreate(lux_task, "lux_task", 3072, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create lux task");
    }
#endif

    esp_err_t wifi_err = wifi_start(on_wifi_ready);

    if (wifi_err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("WIFI", "WiFi not configured; provisioning required");
    } else if (wifi_err != ESP_OK) {
        ESP_LOGE("WIFI", "WiFi start failed: %s", esp_err_to_name(wifi_err));
    }
}
