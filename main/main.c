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
 * CALIBRATION PROCEDURE
 * ─────────────────────
 * 1. Hold the STOP touch pad for 3 seconds.
 *    The LED starts blinking rapidly to confirm calibration mode.
 *
 * 2. Phase 1 – homing close:
 *    The motor runs CLOSED for (cal_ms × 1.5). The motor's own end-stop
 *    stops it; the extra time is harmless.
 *    LED blinks fast during this phase.
 *
 * 3. Phase 2 – open measurement:
 *    The motor starts running OPEN while a timer counts.
 *    LED blinks slowly. Watch the sunshade.
 *
 * 4. When the sunshade is fully open, press the STOP touch pad once.
 *    The elapsed time is saved to NVS as the calibrated travel time.
 *    LED blinks 5× quickly to confirm success.
 *
 * After calibration every HomeKit position (0–100 %) uses this measured time.
 *
 * POWER-LOSS RECOVERY (requires prior calibration)
 * ──────────────────────────────────────────────────
 * On every boot the sunshade first runs fully CLOSED (end-stop guarantees
 * position 0 %). It then moves to the last target position that HomeKit set.
 */

#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <nvs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/touch_pad.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include "esp32-lcm.h"
#include <button.h>

// ── GPIO pin assignments ───────────────────────────────────────────────────────
#define LED_GPIO          CONFIG_ESP_LED_GPIO
#define BUTTON_GPIO       CONFIG_ESP_BUTTON_GPIO
#define RELAY_OPEN_GPIO   CONFIG_ESP_RELAY_OPEN_GPIO
#define RELAY_CLOSE_GPIO  CONFIG_ESP_RELAY_CLOSE_GPIO

// Capacitive touch channels (ESP32 GPIO→Touch mapping is fixed in silicon):
//   GPIO27 = T7,  GPIO33 = T8,  GPIO32 = T9
#define TOUCH_UP_PAD    TOUCH_PAD_NUM9   // GPIO32
#define TOUCH_STOP_PAD  TOUCH_PAD_NUM8   // GPIO33
#define TOUCH_DOWN_PAD  TOUCH_PAD_NUM7   // GPIO27

// ── Timing ────────────────────────────────────────────────────────────────────
// Default travel time from Kconfig; replaced by NVS value after calibration.
#define DEFAULT_TRAVEL_MS       CONFIG_SUNSHADE_FULL_TRAVEL_TIME_MS
#define POS_UPDATE_INTERVAL_MS  200

// End-stop runs (homing / calibration phase 1) use a safety buffer so the
// motor always reaches the physical stop even if the saved time is slightly off.
#define ENDSTOP_BUFFER_FACTOR   1.5f

// Maximum time the user has to confirm "fully open" during calibration phase 2.
// After this the calibration is aborted.
#define CAL_OPEN_MAX_MS         120000u   // 2 minutes

// Touch: pad is considered touched when its reading drops below baseline × factor.
#define TOUCH_THRESHOLD_FACTOR  0.70f
#define TOUCH_POLL_MS           50u
#define CAL_HOLD_TRIGGER_MS     3000u     // hold STOP this long to enter calibration

// ── NVS ───────────────────────────────────────────────────────────────────────
#define NVS_NS          "shade"
#define NVS_CAL_DONE    "cal_done"   // u8 – 1 if calibrated
#define NVS_CAL_MS      "cal_ms"     // u32 – measured travel time in ms
#define NVS_LAST_POS    "last_pos"   // u8 – last HomeKit target position (0–100)

// ── Log tags ──────────────────────────────────────────────────────────────────
static const char *TAG        = "SUNSHADE";
static const char *TAG_TOUCH  = "TOUCH";
static const char *TAG_BUTTON = "BUTTON";
static const char *TAG_RELAY  = "RELAY";
static const char *TAG_CAL    = "CAL";
static const char *TAG_NVS    = "NVS";

// ── Motion state ──────────────────────────────────────────────────────────────
// Enum values match the HomeKit POSITION_STATE characteristic exactly.
typedef enum {
    MOTION_CLOSING = 0,   // HomeKit: DECREASING
    MOTION_OPENING = 1,   // HomeKit: INCREASING
    MOTION_STOPPED = 2,   // HomeKit: STOPPED
} motion_dir_t;

static volatile int          s_cur_pos   = 0;
static volatile int          s_tgt_pos   = 0;
static volatile motion_dir_t s_motion    = MOTION_STOPPED;
static TaskHandle_t          s_move_task = NULL;

// ── Calibration state ─────────────────────────────────────────────────────────
typedef enum {
    CAL_IDLE    = 0,
    CAL_CLOSING,    // phase 1: running to fully closed before measuring
    CAL_OPENING,    // phase 2: running open while the user watches
} cal_state_t;

static volatile cal_state_t s_cal_state  = CAL_IDLE;
static volatile bool        s_cal_done   = false;
static volatile uint32_t    s_cal_ms     = DEFAULT_TRAVEL_MS;

// Timestamp (ms since boot) when phase 2 started.
static volatile uint32_t    s_cal_t0_ms  = 0;

// Set while the boot homing sequence is running; blocks external commands.
static volatile bool        s_is_homing  = false;

// ── Forward declarations ───────────────────────────────────────────────────────
static void sunshade_stop(void);
static void sunshade_move_to(int target);
static void relays_all_off(void);
static void homekit_notify_position(void);
static void calibration_confirm_open(void);

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
    nvs_get_u8(h, NVS_CAL_DONE, &done);
    if (done) {
        uint32_t ms = DEFAULT_TRAVEL_MS;
        nvs_get_u32(h, NVS_CAL_MS, &ms);
        s_cal_ms   = ms;
        s_cal_done = true;
        ESP_LOGI(TAG_NVS, "Calibration loaded: %lu ms", (unsigned long)ms);
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
    nvs_set_u8(h,  NVS_CAL_DONE, 1);
    nvs_set_u32(h, NVS_CAL_MS,   travel_ms);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG_NVS, "Calibration saved: %lu ms", (unsigned long)travel_ms);
}

static uint8_t nvs_load_last_position(void) {
    nvs_handle_t h;
    uint8_t pos = 0;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return 0;
    nvs_get_u8(h, NVS_LAST_POS, &pos);
    nvs_close(h);
    ESP_LOGI(TAG_NVS, "Last saved position: %d%%", pos);
    return pos;
}

// Save the intended target so it can be restored after a power loss.
static void nvs_save_last_position(uint8_t pos) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, NVS_LAST_POS, pos);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGD(TAG_NVS, "Saved target position: %d%%", pos);
}

// ── LED ───────────────────────────────────────────────────────────────────────
static void led_write(bool on) {
    gpio_set_level(LED_GPIO, on ? 1 : 0);
}

// ── Relay control (software interlock) ────────────────────────────────────────
static void relays_all_off(void) {
    gpio_set_level(RELAY_OPEN_GPIO,  0);
    gpio_set_level(RELAY_CLOSE_GPIO, 0);
    ESP_LOGD(TAG_RELAY, "Both relays OFF");
}

static void relay_activate_open(void) {
    gpio_set_level(RELAY_CLOSE_GPIO, 0);   // interlock: close OFF before open ON
    gpio_set_level(RELAY_OPEN_GPIO,  1);
    ESP_LOGD(TAG_RELAY, "OPEN relay ON");
}

static void relay_activate_close(void) {
    gpio_set_level(RELAY_OPEN_GPIO,  0);   // interlock: open OFF before close ON
    gpio_set_level(RELAY_CLOSE_GPIO, 1);
    ESP_LOGD(TAG_RELAY, "CLOSE relay ON");
}

// ── HomeKit characteristics ────────────────────────────────────────────────────
static void target_position_setter(homekit_value_t value);

static homekit_characteristic_t current_pos_ch =
    HOMEKIT_CHARACTERISTIC_(CURRENT_POSITION, 0);

static homekit_characteristic_t target_pos_ch =
    HOMEKIT_CHARACTERISTIC_(TARGET_POSITION, 0,
        .setter = target_position_setter);

static homekit_characteristic_t pos_state_ch =
    HOMEKIT_CHARACTERISTIC_(POSITION_STATE, 2);   // 2 = STOPPED

static void homekit_notify_position(void) {
    current_pos_ch.value = HOMEKIT_UINT8((uint8_t)s_cur_pos);
    homekit_characteristic_notify(&current_pos_ch, current_pos_ch.value);

    pos_state_ch.value = HOMEKIT_UINT8((uint8_t)s_motion);
    homekit_characteristic_notify(&pos_state_ch, pos_state_ch.value);
}

// ── Movement task ──────────────────────────────────────────────────────────────
// Time-based position tracking. Uses s_cal_ms (calibrated or default).
// The task re-syncs its internal accumulator when direction changes mid-motion.
// Relays must already be set before this task is started.

static void movement_task(void *arg) {
    // Snapshot the travel time at task start so calibration changes mid-run
    // do not corrupt position tracking.
    const uint32_t travel_ms = s_cal_ms;
    const float    delta     = 100.0f * POS_UPDATE_INTERVAL_MS / (float)travel_ms;

    motion_dir_t last_dir = s_motion;
    float        pos_f    = (float)s_cur_pos;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POS_UPDATE_INTERVAL_MS));

        motion_dir_t dir = s_motion;

        // Re-sync accumulator on direction change (e.g. reverse mid-travel).
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
        } else {                            // MOTION_CLOSING
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

        s_cur_pos = (int)pos_f;
        homekit_notify_position();
    }

    s_move_task = NULL;
    vTaskDelete(NULL);
}

static void movement_task_start(void) {
    if (s_move_task != NULL) return;   // running task picks up new state
    xTaskCreate(movement_task, "sunshade_move", 4096, NULL, 5, &s_move_task);
}

// ── Sunshade control ──────────────────────────────────────────────────────────
// All control functions guard against calibration and homing.

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
    if (is_locked()) return;
    if (s_cur_pos >= 100) { ESP_LOGI(TAG, "Already fully open"); return; }
    ESP_LOGI(TAG, "Opening: %d%% → 100%%", s_cur_pos);
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
    if (is_locked()) return;
    if (s_cur_pos <= 0) { ESP_LOGI(TAG, "Already fully closed"); return; }
    ESP_LOGI(TAG, "Closing: %d%% → 0%%", s_cur_pos);
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
    // During calibration phase 2, a quick STOP press confirms full open.
    if (s_cal_state == CAL_OPENING) {
        calibration_confirm_open();
        return;
    }
    if (is_locked()) return;

    ESP_LOGI(TAG, "Stop at %d%%", s_cur_pos);
    s_motion            = MOTION_STOPPED;
    s_tgt_pos           = s_cur_pos;
    target_pos_ch.value = HOMEKIT_UINT8((uint8_t)s_cur_pos);
    relays_all_off();
    homekit_notify_position();
    nvs_save_last_position((uint8_t)s_cur_pos);
}

static void sunshade_move_to(int target) {
    if (is_locked()) return;
    if (target < 0)   target = 0;
    if (target > 100) target = 100;

    if (target == s_cur_pos) { sunshade_stop(); return; }

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

// ── HomeKit setter ────────────────────────────────────────────────────────────
static void target_position_setter(homekit_value_t value) {
    if (value.format != homekit_format_uint8) {
        ESP_LOGE(TAG, "target_position: unexpected format %d", value.format);
        return;
    }
    ESP_LOGI(TAG, "HomeKit → target_position: %d%%", value.uint8_value);
    sunshade_move_to((int)value.uint8_value);
}

// ── Calibration ───────────────────────────────────────────────────────────────
//
// Runs in its own FreeRTOS task so the blocking close-run does not stall the
// touch task.
//
// Phase 1: motor runs CLOSE for (cal_ms × ENDSTOP_BUFFER_FACTOR); motor's
//          mechanical end-stop guarantees position 0 %.
// Phase 2: motor runs OPEN while a counter measures elapsed time. User presses
//          STOP when the sunshade is fully open. Elapsed time is saved.
//
// calibration_confirm_open() is called by the touch task (STOP quick press
// while s_cal_state == CAL_OPENING).

static void calibration_led_task(void *arg) {
    // Blink LED while calibration is active.
    // Fast during phase 1 (closing), slow during phase 2 (opening).
    while (s_cal_state != CAL_IDLE) {
        uint32_t half = (s_cal_state == CAL_CLOSING) ? 100 : 400;
        led_write(true);
        vTaskDelay(pdMS_TO_TICKS(half));
        led_write(false);
        vTaskDelay(pdMS_TO_TICKS(half));
    }
    // 5 quick blinks on success / abort.
    for (int i = 0; i < 5; i++) {
        led_write(true);  vTaskDelay(pdMS_TO_TICKS(60));
        led_write(false); vTaskDelay(pdMS_TO_TICKS(60));
    }
    vTaskDelete(NULL);
}

// Called from the touch task when STOP is quickly pressed during CAL_OPENING.
static void calibration_confirm_open(void) {
    uint32_t elapsed = now_ms() - s_cal_t0_ms;

    relays_all_off();

    if (elapsed < 2000) {
        ESP_LOGW(TAG_CAL, "Aborted: STOP pressed too soon (%lu ms)", (unsigned long)elapsed);
        s_cal_state = CAL_IDLE;
        s_cur_pos   = 0;
        return;
    }

    s_cal_ms   = elapsed;
    s_cal_done = true;
    s_cur_pos  = 100;
    s_tgt_pos  = 100;
    s_motion   = MOTION_STOPPED;

    current_pos_ch.value = HOMEKIT_UINT8(100);
    target_pos_ch.value  = HOMEKIT_UINT8(100);
    s_cal_state          = CAL_IDLE;   // signals calibration_task to exit

    nvs_save_calibration(elapsed);
    nvs_save_last_position(100);
    homekit_notify_position();

    ESP_LOGI(TAG_CAL, "Calibration complete: travel time = %lu ms", (unsigned long)elapsed);
    ESP_LOGI(TAG_CAL, "Sunshade is now at 100%%. HomeKit positions 0–100%% are active.");
}

static void calibration_task(void *arg) {
    ESP_LOGI(TAG_CAL, "=== CALIBRATION START ===");

    // Stop any in-progress motion.
    s_motion = MOTION_STOPPED;
    relays_all_off();
    vTaskDelay(pdMS_TO_TICKS(300));

    // ── Phase 1: close fully ──────────────────────────────────────────────────
    ESP_LOGI(TAG_CAL, "Phase 1: closing fully...");
    s_cal_state = CAL_CLOSING;

    uint32_t close_ms = (uint32_t)(s_cal_ms * ENDSTOP_BUFFER_FACTOR);
    if (close_ms < 8000) close_ms = 8000;   // never less than 8 s

    relay_activate_close();
    vTaskDelay(pdMS_TO_TICKS(close_ms));
    relays_all_off();

    s_cur_pos = 0;
    current_pos_ch.value = HOMEKIT_UINT8(0);
    homekit_notify_position();

    vTaskDelay(pdMS_TO_TICKS(500));   // pause before starting open run

    // ── Phase 2: open and measure ─────────────────────────────────────────────
    ESP_LOGI(TAG_CAL, "Phase 2: opening – press STOP touch when fully open!");
    s_cal_t0_ms = now_ms();
    s_cal_state = CAL_OPENING;
    relay_activate_open();

    // Wait for calibration_confirm_open() to be called, or for timeout.
    uint32_t t_start = now_ms();
    while (s_cal_state == CAL_OPENING) {
        if ((now_ms() - t_start) > CAL_OPEN_MAX_MS) {
            ESP_LOGW(TAG_CAL, "Timeout: calibration aborted after %lu s",
                     (unsigned long)(CAL_OPEN_MAX_MS / 1000));
            relays_all_off();
            s_cal_state = CAL_IDLE;
            s_cur_pos   = 0;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG_CAL, "=== CALIBRATION END (state: %s) ===",
             s_cal_done ? "SUCCESS" : "ABORTED");
    vTaskDelete(NULL);
}

// Called from touch_task when STOP is held for CAL_HOLD_TRIGGER_MS.
static void calibration_start(void) {
    if (s_cal_state != CAL_IDLE) return;
    if (s_is_homing) {
        ESP_LOGW(TAG_CAL, "Cannot calibrate while homing");
        return;
    }
    xTaskCreate(calibration_led_task, "cal_led",  1024, NULL, 3, NULL);
    xTaskCreate(calibration_task,     "cal_task", 4096, NULL, 4, NULL);
}

// ── Boot homing ───────────────────────────────────────────────────────────────
// Runs once after boot (in its own task) if the device is calibrated.
// Closes the sunshade fully to establish position 0 %, then restores the
// last HomeKit target.

static void homing_task(void *arg) {
    uint8_t last_pos = (uint8_t)(uintptr_t)arg;

    ESP_LOGI(TAG, "Homing: closing fully to establish position 0%%...");
    s_is_homing = true;
    s_motion    = MOTION_CLOSING;

    uint32_t close_ms = (uint32_t)(s_cal_ms * ENDSTOP_BUFFER_FACTOR);
    if (close_ms < 8000) close_ms = 8000;

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

    s_is_homing = false;   // unlock commands before moving

    if (last_pos > 0 && last_pos <= 100) {
        ESP_LOGI(TAG, "Homing: restoring to last target %d%%", last_pos);
        sunshade_move_to((int)last_pos);
    } else {
        ESP_LOGI(TAG, "Homing complete; sunshade at 0%%");
    }

    vTaskDelete(NULL);
}

// ── Identify ──────────────────────────────────────────────────────────────────
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
    vTaskDelete(NULL);
}

static void accessory_identify(homekit_value_t _value) {
    ESP_LOGI("INFORMATION", "Accessory identify");
    xTaskCreate(accessory_identify_task, "identify", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
}

// ── HomeKit accessory definition ──────────────────────────────────────────────
#define DEVICE_NAME         "HomeKit Sunshade"
#define DEVICE_MANUFACTURER "StudioPieters\xc2\xae"
#define DEVICE_SERIAL       "NLDA4SQN1466"
#define DEVICE_MODEL        "SD466NL/A"

static homekit_characteristic_t name         = HOMEKIT_CHARACTERISTIC_(NAME, DEVICE_NAME);
static homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER, DEVICE_MANUFACTURER);
static homekit_characteristic_t serial       = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, DEVICE_SERIAL);
static homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL, DEVICE_MODEL);
static homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION, LIFECYCLE_DEFAULT_FW_VERSION);
static homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;

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

// ── Touch pads ────────────────────────────────────────────────────────────────
static uint16_t touch_base_up   = 0;
static uint16_t touch_base_stop = 0;
static uint16_t touch_base_down = 0;

static void touch_init(void) {
    ESP_ERROR_CHECK(touch_pad_init());
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_UP_PAD,   0));
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_STOP_PAD, 0));
    ESP_ERROR_CHECK(touch_pad_config(TOUCH_DOWN_PAD, 0));
    ESP_ERROR_CHECK(touch_pad_filter_start(10));   // 10 ms IIR filter

    vTaskDelay(pdMS_TO_TICKS(500));   // stabilise readings

    ESP_ERROR_CHECK(touch_pad_read_filtered(TOUCH_UP_PAD,   &touch_base_up));
    ESP_ERROR_CHECK(touch_pad_read_filtered(TOUCH_STOP_PAD, &touch_base_stop));
    ESP_ERROR_CHECK(touch_pad_read_filtered(TOUCH_DOWN_PAD, &touch_base_down));

    ESP_LOGI(TAG_TOUCH, "Baselines – UP(T9/GPIO32):%u  STOP(T8/GPIO33):%u  DOWN(T7/GPIO27):%u",
             touch_base_up, touch_base_stop, touch_base_down);
}

static void touch_task(void *arg) {
    bool     up_prev   = false;
    bool     stop_prev = false;
    bool     down_prev = false;
    uint32_t stop_held = 0;    // accumulated hold time in ms
    bool     cal_armed = false; // prevents re-triggering while still held
    uint16_t val;

    for (;;) {
        touch_pad_read_filtered(TOUCH_UP_PAD, &val);
        bool up_now = val < (uint16_t)(touch_base_up * TOUCH_THRESHOLD_FACTOR);

        touch_pad_read_filtered(TOUCH_STOP_PAD, &val);
        bool stop_now = val < (uint16_t)(touch_base_stop * TOUCH_THRESHOLD_FACTOR);

        touch_pad_read_filtered(TOUCH_DOWN_PAD, &val);
        bool down_now = val < (uint16_t)(touch_base_down * TOUCH_THRESHOLD_FACTOR);

        // UP – rising edge only
        if (up_now && !up_prev) {
            ESP_LOGI(TAG_TOUCH, "UP touched → opening");
            sunshade_open();
        }

        // DOWN – rising edge only
        if (down_now && !down_prev) {
            ESP_LOGI(TAG_TOUCH, "DOWN touched → closing");
            sunshade_close();
        }

        // STOP – hold detection for calibration; quick release for stop/confirm
        if (stop_now) {
            stop_held += TOUCH_POLL_MS;
            if (!cal_armed && stop_held >= CAL_HOLD_TRIGGER_MS) {
                ESP_LOGI(TAG_TOUCH, "STOP held %lu ms → calibration trigger",
                         (unsigned long)stop_held);
                cal_armed = true;
                calibration_start();
            }
        } else {
            if (stop_prev && !cal_armed) {
                // Quick release – stop or confirm open
                if (s_cal_state == CAL_OPENING) {
                    ESP_LOGI(TAG_TOUCH, "STOP → confirm fully open");
                    calibration_confirm_open();
                } else {
                    ESP_LOGI(TAG_TOUCH, "STOP touched → stopping");
                    sunshade_stop();
                }
            }
            stop_held = 0;
            cal_armed = false;
        }

        up_prev   = up_now;
        stop_prev = stop_now;
        down_prev = down_now;

        vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_MS));
    }
}

// ── Physical button ───────────────────────────────────────────────────────────
static void button_callback(button_event_t event, void *context) {
    switch (event) {
    case button_event_single_press:
        ESP_LOGI(TAG_BUTTON, "Single press → update/reboot");
        relays_all_off();
        lifecycle_request_update_and_reboot();
        break;

    case button_event_double_press:
        ESP_LOGI(TAG_BUTTON, "Double press → HomeKit reset + restart");
        relays_all_off();
        homekit_server_reset();
        esp_restart();
        break;

    case button_event_long_press:
        ESP_LOGI(TAG_BUTTON, "Long press → factory reset + reboot");
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

    ESP_LOGI(TAG, "GPIO: LED=%d  RELAY_OPEN=%d  RELAY_CLOSE=%d",
             LED_GPIO, RELAY_OPEN_GPIO, RELAY_CLOSE_GPIO);
}

// ── WiFi ready ────────────────────────────────────────────────────────────────
static void on_wifi_ready(void) {
    static bool homekit_started = false;
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

    // Load calibration data before anything else reads s_cal_ms.
    nvs_load_calibration();

    // Touch pads must be initialised before the touch task is created.
    touch_init();
    xTaskCreate(touch_task, "touch_task", 2048, NULL, 5, NULL);

    // Physical button (GPIO25, GND→button→GPIO, active-low).
    button_config_t btn_cfg    = button_config_default(button_active_low);
    btn_cfg.max_repeat_presses = 3;
    btn_cfg.long_press_time    = 1000;
    if (button_create(BUTTON_GPIO, btn_cfg, button_callback, NULL)) {
        ESP_LOGE(TAG_BUTTON, "Failed to init button on GPIO%d", BUTTON_GPIO);
    }

    // Boot homing: only when device is calibrated.
    // Runs in a background task so WiFi/HomeKit init is not delayed.
    if (s_cal_done) {
        uint8_t last_pos = nvs_load_last_position();
        ESP_LOGI(TAG, "Calibrated: starting homing sequence (last pos: %d%%)", last_pos);
        xTaskCreate(homing_task, "homing", 4096,
                    (void *)(uintptr_t)last_pos, 4, NULL);
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
