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

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include "esp32-lcm.h"
#include <button.h>

// ── GPIO pin assignments ───────────────────────────────────────────────────────
#define LED_GPIO            CONFIG_ESP_LED_GPIO
#define BUTTON_GPIO         CONFIG_ESP_BUTTON_GPIO
#define RELAY_OPEN_GPIO     CONFIG_ESP_RELAY_OPEN_GPIO
#define RELAY_CLOSE_GPIO    CONFIG_ESP_RELAY_CLOSE_GPIO

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

// ── Log tags ──────────────────────────────────────────────────────────────────
static const char *TAG        = "SUNSHADE";
static const char *TAG_TOUCH  = "TTP223";
static const char *TAG_BUTTON = "BUTTON";
static const char *TAG_RELAY  = "RELAY";
static const char *TAG_CAL    = "CAL";
static const char *TAG_NVS    = "NVS";

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
static void relays_all_off(void) {
    gpio_set_level(RELAY_OPEN_GPIO,  0);
    gpio_set_level(RELAY_CLOSE_GPIO, 0);
    ESP_LOGD(TAG_RELAY, "Both relays OFF");
}

static void relay_activate_open(void) {
    gpio_set_level(RELAY_CLOSE_GPIO, 0);
    gpio_set_level(RELAY_OPEN_GPIO,  1);
    ESP_LOGD(TAG_RELAY, "OPEN relay ON");
}

static void relay_activate_close(void) {
    gpio_set_level(RELAY_OPEN_GPIO,  0);
    gpio_set_level(RELAY_CLOSE_GPIO, 1);
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
    const float    delta     = 100.0f * POS_UPDATE_INTERVAL_MS / (float)travel_ms;

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

        int new_pos = (int)pos_f;
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
    gpio_set_level(RELAY_OPEN_GPIO, 0);

    gpio_reset_pin(RELAY_CLOSE_GPIO);
    gpio_set_direction(RELAY_CLOSE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_CLOSE_GPIO, 0);

    ESP_LOGI(TAG, "GPIO: LED=%d RELAY_OPEN=%d RELAY_CLOSE=%d",
             LED_GPIO, RELAY_OPEN_GPIO, RELAY_CLOSE_GPIO);
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

    esp_err_t wifi_err = wifi_start(on_wifi_ready);

    if (wifi_err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("WIFI", "WiFi not configured; provisioning required");
    } else if (wifi_err != ESP_OK) {
        ESP_LOGE("WIFI", "WiFi start failed: %s", esp_err_to_name(wifi_err));
    }
}
