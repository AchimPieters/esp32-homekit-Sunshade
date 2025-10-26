/**
 *  HomeKit Sunshade – ESP32
 *  - Optional HomeKit "Recalibrate" switch
 *  - Button double-click (STOP -> 50%)
 *  - Calibration with NVS persistence
 *  - NeoPixel status animations
 *
 *  Copyright 2025 Achim Pieters | StudioPieters®
 *  Licensed as in original header…
 *  More info: https://www.studiopieters.nl
 *
 *  Notes:
 *  - This file expects the Kconfig you supplied (classic ESP32 ranges 0..39).
 *  - For ESP32-S3 / ESP32-C3 recommended pin maps are listed below as comments.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <math.h>              // sinf
#include <esp_log.h>
#include <esp_err.h>
#include <esp_check.h>         // ESP_RETURN_ON_ERROR
#include <esp_timer.h>         // esp_timer_get_time
#include <nvs.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include "esp32-lcm.h"
#include <button.h>            // achimpieters/esp32-button

// NeoPixel (WS2812) via ESP-IDF led_strip (RMT)
#include "led_strip.h"
#include "driver/rmt_tx.h"

// -----------------------------------------------------------------------------
// Pin configuration through Kconfig (classic ESP32 ranges 0..39)
// -----------------------------------------------------------------------------
#define LED_GPIO                  CONFIG_ESP_LED_GPIO     // Identify LED (separate from NeoPixel)

// Relays
#define RELAY_OPEN_GPIO           CONFIG_ESP_OPEN_GPIO
#define RELAY_CLOSE_GPIO          CONFIG_ESP_CLOSE_GPIO
#define RELAY_ACTIVE_LEVEL        CONFIG_RELAY_ACTIVE_LEVEL   // 1 = active HIGH, 0 = active LOW

// Buttons (esp32-button component)
#define BTN_OPEN_GPIO             CONFIG_BTN_OPEN_GPIO
#define BTN_STOP_GPIO             CONFIG_BTN_STOP_GPIO
#define BTN_CLOSE_GPIO            CONFIG_BTN_CLOSE_GPIO
#ifndef CONFIG_BUTTON_ACTIVE_LEVEL
#define CONFIG_BUTTON_ACTIVE_LEVEL 0                           // default: active LOW (to GND with internal pull-up)
#endif

// NeoPixel
#define NEOPIXEL_GPIO             CONFIG_NEOPIXEL_GPIO
#define NEOPIXEL_LED_COUNT        1

// Motion model (fallback). Will be overridden after calibration load from NVS.
#define FULL_TRAVEL_MS_DEFAULT    18000
#define MOVE_TICK_MS              100
#define MID_POSITION              50     // used for STOP double-click

// -----------------------------------------------------------------------------
// (Optional) Target-specific pin map guidance (informational)
// -----------------------------------------------------------------------------
// ESP32-S3 (if you plan to target it):
//   Safe outputs: 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,38,39,40,41,42,45,46
//   Strapping pins: 0, 3, 45, 46 (avoid for buttons/relays unless you know the boot implications)
//   Suggested defaults: RELAY 16/17, Buttons 8/9/10, NeoPixel 18, LED 13
//
// ESP32-C3:
//   Valid GPIOs: 0..10,18,19
//   Strapping pins: 2,8 (be careful), 0 is boot mode (avoid fixed pulls)
//   Suggested defaults: RELAY 6/7, Buttons 3/4/5, NeoPixel 8 or 10, LED 2
//
// Your provided Kconfig keeps ranges 0..39 for classic ESP32; adjust Kconfig
// per target if you decide to support S3/C3 in the same project tree.

// -----------------------------------------------------------------------------
// Identify LED (GPIO) – separate from NeoPixel animations
// -----------------------------------------------------------------------------
static bool led_on = false;
static inline void led_write(bool on) {
        gpio_set_level(LED_GPIO, on ? 1 : 0);
}
static void gpio_init_led(void) {
        gpio_reset_pin(LED_GPIO);
        gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
        led_write(led_on);
}

// -----------------------------------------------------------------------------
// Relays (with interlock)
// -----------------------------------------------------------------------------
static const char *MOTOR_TAG = "MOTOR";

static inline void relay_write(gpio_num_t pin, bool on) {
        gpio_set_level(pin, (on ? RELAY_ACTIVE_LEVEL : !RELAY_ACTIVE_LEVEL));
}
static void motor_all_off(void) {
        relay_write(RELAY_OPEN_GPIO, false);
        relay_write(RELAY_CLOSE_GPIO, false);
}
static void motor_drive_open(bool on) {
        if (on) relay_write(RELAY_CLOSE_GPIO, false);
        relay_write(RELAY_OPEN_GPIO, on);
}
static void motor_drive_close(bool on) {
        if (on) relay_write(RELAY_OPEN_GPIO, false);
        relay_write(RELAY_CLOSE_GPIO, on);
}
static void gpio_init_motor(void) {
        gpio_reset_pin(RELAY_OPEN_GPIO);
        gpio_reset_pin(RELAY_CLOSE_GPIO);
        gpio_set_direction(RELAY_OPEN_GPIO, GPIO_MODE_OUTPUT);
        gpio_set_direction(RELAY_CLOSE_GPIO, GPIO_MODE_OUTPUT);
        motor_all_off();
}

// -----------------------------------------------------------------------------
// NeoPixel (status)
// -----------------------------------------------------------------------------
static led_strip_handle_t s_strip = NULL;

static void neopixel_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
        if (!s_strip) return;
        led_strip_set_pixel(s_strip, 0, r, g, b);
        led_strip_refresh(s_strip);
}
static void neopixel_off(void) {
        neopixel_set_rgb(0, 0, 0);
}

typedef enum {
        PIX_IDLE = 0,
        PIX_OPENING,
        PIX_CLOSING,
        PIX_STOPPED,
        PIX_CALIBRATING,
        PIX_WIFI_WAIT
} pix_state_t;

static pix_state_t pix_state = PIX_WIFI_WAIT;

// Simple breathing animation; task period = 50ms
static void neopixel_anim_task(void *arg) {
        uint8_t t = 0;
        while (1) {
                switch (pix_state) {
                case PIX_WIFI_WAIT: { // Orange breathing
                        uint8_t v = (uint8_t)(8 + (7 * (1 + sinf(t / 10.0f))));
                        neopixel_set_rgb(v * 2, v, 0);
                } break;
                case PIX_CALIBRATING: { // Purple breathing
                        uint8_t v = (uint8_t)(6 + (6 * (1 + sinf(t / 8.0f))));
                        neopixel_set_rgb(v, 0, v * 2);
                } break;
                case PIX_OPENING:  neopixel_set_rgb(0, 40, 0);  break;// Green solid
                case PIX_CLOSING:  neopixel_set_rgb(40, 0, 0);  break;// Red solid
                case PIX_STOPPED:  neopixel_set_rgb(0, 25, 25); break;// Cyan solid
                case PIX_IDLE:     neopixel_set_rgb(8, 8, 8);   break;// Soft white
                }
                t++;
                vTaskDelay(pdMS_TO_TICKS(50));
        }
}

static void neopixel_init(void) {
        // In ESP-IDF 5.x, led_strip_* creates/owns the RMT channel; no need to allocate one manually.
        led_strip_config_t strip_config = {
                .strip_gpio_num = NEOPIXEL_GPIO,
                .max_leds = NEOPIXEL_LED_COUNT,
        };
        led_strip_rmt_config_t rmt_config = {
                .resolution_hz = 10 * 1000 * 1000, // 10 MHz
                .flags = { .with_dma = false },
        };
        ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip));
        ESP_ERROR_CHECK(led_strip_clear(s_strip)); // off
        xTaskCreate(neopixel_anim_task, "np_anim", 2048, NULL, 1, NULL);
}

// -----------------------------------------------------------------------------
// Identify – blink both GPIO LED and NeoPixel
// -----------------------------------------------------------------------------
static void accessory_identify_task(void *args) {
        for (int i = 0; i < 3; i++) {
                neopixel_set_rgb(20, 20, 20);
                led_write(true);
                vTaskDelay(pdMS_TO_TICKS(150));
                neopixel_off();
                led_write(false);
                vTaskDelay(pdMS_TO_TICKS(150));
        }
        led_write(led_on);
        vTaskDelete(NULL);
}
static void accessory_identify(homekit_value_t _value) {
        ESP_LOGI("INFO", "Accessory identify");
        xTaskCreate(accessory_identify_task, "identify", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
}

// -----------------------------------------------------------------------------
// HomeKit metadata
// -----------------------------------------------------------------------------
#define DEVICE_NAME          "HomeKit Sunshade"
#define DEVICE_MANUFACTURER  "StudioPieters®"
#define DEVICE_SERIAL        "Y3GLW8G950FW"
#define DEVICE_MODEL         "VB14B1CA/H"

homekit_characteristic_t name         = HOMEKIT_CHARACTERISTIC_(NAME, DEVICE_NAME);
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER, DEVICE_MANUFACTURER);
homekit_characteristic_t serial       = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, DEVICE_SERIAL);
homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL, DEVICE_MODEL);
homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION, LIFECYCLE_DEFAULT_FW_VERSION);
homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;

// -----------------------------------------------------------------------------
// Window Covering state + optional Recalibrate switch
// -----------------------------------------------------------------------------
/*
 * HomeKit:
 * CurrentPosition:   0 = closed, 100 = open
 * TargetPosition:    0..100
 * PositionState:     0=DECREASING (closing)  1=INCREASING (opening)  2=STOPPED
 */
static TaskHandle_t move_task_handle = NULL;
static float pos_f = 0.0f; // internal position 0..100
static uint32_t full_travel_ms = FULL_TRAVEL_MS_DEFAULT;

static homekit_characteristic_t current_position =
        HOMEKIT_CHARACTERISTIC_(CURRENT_POSITION, 0);
static homekit_characteristic_t target_position =
        HOMEKIT_CHARACTERISTIC_(TARGET_POSITION, 0);
static homekit_characteristic_t position_state =
        HOMEKIT_CHARACTERISTIC_(POSITION_STATE, 2);
static homekit_characteristic_t obstruction_detected =
        HOMEKIT_CHARACTERISTIC_(OBSTRUCTION_DETECTED, false);
static homekit_characteristic_t hold_position =
        HOMEKIT_CHARACTERISTIC_(HOLD_POSITION, false);

// Optional HomeKit "Recalibrate" (momentary) switch characteristic
static void hk_recal_switch_set(homekit_value_t value);
static homekit_characteristic_t recalibrate_switch =
        HOMEKIT_CHARACTERISTIC_(ON, false, .setter = hk_recal_switch_set);

// Forward decl
static void start_move_to(uint8_t target);

// -----------------------------------------------------------------------------
// NVS helpers (calibration storage)
// -----------------------------------------------------------------------------
static esp_err_t calib_load(uint32_t *out_ms) {
        nvs_handle_t nvh;
        esp_err_t err = nvs_open("sunshade", NVS_READONLY, &nvh);
        if (err != ESP_OK) return err;
        uint32_t v = 0;
        err = nvs_get_u32(nvh, "full_ms", &v);
        nvs_close(nvh);
        if (err == ESP_OK && v >= 3000 && v <= 120000) { *out_ms = v; return ESP_OK; }
        return ESP_ERR_INVALID_STATE;
}
static esp_err_t calib_save(uint32_t ms) {
        nvs_handle_t nvh;
        ESP_RETURN_ON_ERROR(nvs_open("sunshade", NVS_READWRITE, &nvh), "CAL", "NVS open");
        ESP_ERROR_CHECK(nvs_set_u32(nvh, "full_ms", ms));
        ESP_ERROR_CHECK(nvs_commit(nvh));
        nvs_close(nvh);
        return ESP_OK;
}

// -----------------------------------------------------------------------------
// Movement task
// -----------------------------------------------------------------------------
static void move_task(void *param) {
        uint8_t tgt = (uint8_t)(intptr_t)param;
        const float step = (100.0f * (float)MOVE_TICK_MS) / (float)full_travel_ms; // % per tick

        if (tgt > (uint8_t)pos_f) {
                position_state.value = HOMEKIT_UINT8(1); // opening
                homekit_characteristic_notify(&position_state, position_state.value);
                pix_state = PIX_OPENING;
                motor_drive_open(true);
                while (pos_f < tgt && position_state.value.uint8_value == 1) {
                        pos_f += step;
                        if (pos_f > 100.0f) pos_f = 100.0f;
                        uint8_t cp = (uint8_t)(pos_f + 0.5f);
                        if (cp != current_position.value.uint8_value) {
                                current_position.value = HOMEKIT_UINT8(cp);
                                homekit_characteristic_notify(&current_position, current_position.value);
                        }
                        vTaskDelay(pdMS_TO_TICKS(MOVE_TICK_MS));
                }
        } else if (tgt < (uint8_t)pos_f) {
                position_state.value = HOMEKIT_UINT8(0); // closing
                homekit_characteristic_notify(&position_state, position_state.value);
                pix_state = PIX_CLOSING;
                motor_drive_close(true);
                while (pos_f > tgt && position_state.value.uint8_value == 0) {
                        pos_f -= step;
                        if (pos_f < 0.0f) pos_f = 0.0f;
                        uint8_t cp = (uint8_t)(pos_f + 0.5f);
                        if (cp != current_position.value.uint8_value) {
                                current_position.value = HOMEKIT_UINT8(cp);
                                homekit_characteristic_notify(&current_position, current_position.value);
                        }
                        vTaskDelay(pdMS_TO_TICKS(MOVE_TICK_MS));
                }
        }

        motor_all_off();
        position_state.value = HOMEKIT_UINT8(2); // stopped
        homekit_characteristic_notify(&position_state, position_state.value);

        // Snap exact target value
        pos_f = tgt;
        if (current_position.value.uint8_value != tgt) {
                current_position.value = HOMEKIT_UINT8(tgt);
                homekit_characteristic_notify(&current_position, current_position.value);
        }

        pix_state = (tgt == 0 || tgt == 100) ? PIX_IDLE : PIX_STOPPED;

        move_task_handle = NULL;
        vTaskDelete(NULL);
}

static void start_move_to(uint8_t target) {
        if (move_task_handle) {
                vTaskDelete(move_task_handle);
                move_task_handle = NULL;
        }
        motor_all_off();

        if (target == (uint8_t)pos_f) {
                position_state.value = HOMEKIT_UINT8(2);
                homekit_characteristic_notify(&position_state, position_state.value);
                pix_state = PIX_STOPPED;
                return;
        }
        xTaskCreate(move_task, "move", 3072, (void *)(intptr_t)target, 5, &move_task_handle);
}

// Convenience helpers that also notify target_position
static void start_move_open(void) {
        if (target_position.value.uint8_value != 100) {
                target_position.value = HOMEKIT_UINT8(100);
                homekit_characteristic_notify(&target_position, target_position.value);
        }
        start_move_to(100);
}
static void start_move_close(void) {
        if (target_position.value.uint8_value != 0) {
                target_position.value = HOMEKIT_UINT8(0);
                homekit_characteristic_notify(&target_position, target_position.value);
        }
        start_move_to(0);
}
static void start_move_mid(void) { // used by STOP double-click
        if (target_position.value.uint8_value != MID_POSITION) {
                target_position.value = HOMEKIT_UINT8(MID_POSITION);
                homekit_characteristic_notify(&target_position, target_position.value);
        }
        start_move_to(MID_POSITION);
}

// -----------------------------------------------------------------------------
// HomeKit setters
// -----------------------------------------------------------------------------
static void target_position_set(homekit_value_t value) {
        if (value.format != homekit_format_uint8) return;
        uint8_t t = value.uint8_value;
        ESP_LOGI(MOTOR_TAG, "New target: %u", t);
        target_position.value = value; // store
        start_move_to(t);
        homekit_characteristic_notify(&target_position, target_position.value);
}

static void hold_position_set(homekit_value_t value) {
        if (value.format != homekit_format_bool) return;
        if (value.bool_value) {
                ESP_LOGI(MOTOR_TAG, "Hold position (STOP)");
                motor_all_off();
                position_state.value = HOMEKIT_UINT8(2);
                homekit_characteristic_notify(&position_state, position_state.value);
                pix_state = PIX_STOPPED;
        }
        hold_position.value = HOMEKIT_BOOL(false); // auto-reset to false
        homekit_characteristic_notify(&hold_position, hold_position.value);
}

// Recalibrate switch setter (momentary behavior)
//  - ON -> if IDLE enter calibration; if already calibrating, cancel
//  - Always auto-resets to OFF
static void hk_recal_switch_set(homekit_value_t value);

// -----------------------------------------------------------------------------
// Calibration state machine
// -----------------------------------------------------------------------------
typedef enum {
        CAL_IDLE = 0,
        CAL_ARMED, // activated via STOP long-press or Recalibrate switch
        CAL_RUNNING // measuring (opening while timing)
} calib_state_t;

static calib_state_t calib_state = CAL_IDLE;
static int64_t calib_start_us = 0;

static void calib_enter(void) {
        if (calib_state != CAL_IDLE) return;
        ESP_LOGI("CAL", "Calibration MODE ON: set shade fully CLOSED, press OPEN to start timing, press STOP when fully OPEN.");
        motor_all_off();
        calib_state = CAL_ARMED;
        pix_state = PIX_CALIBRATING;
}

static void calib_cancel(void) {
        if (calib_state == CAL_IDLE) return;
        ESP_LOGI("CAL", "Calibration CANCELLED");
        motor_all_off();
        calib_state = CAL_IDLE;
        pix_state = PIX_IDLE;
}

static void calib_start_run(void) {
        if (calib_state != CAL_ARMED) return;
        // Assumption: start at fully CLOSED
        ESP_LOGI("CAL", "Calibration START: opening and measuring travel time…");
        calib_start_us = esp_timer_get_time();
        calib_state = CAL_RUNNING;
        // Force fully open movement without altering HomeKit target
        if (move_task_handle) { vTaskDelete(move_task_handle); move_task_handle = NULL; }
        position_state.value = HOMEKIT_UINT8(1);
        homekit_characteristic_notify(&position_state, position_state.value);
        motor_drive_open(true);
        pix_state = PIX_CALIBRATING;
}

static void calib_finish_on_stop(void) {
        if (calib_state != CAL_RUNNING) return;
        motor_all_off();
        int64_t elapsed = esp_timer_get_time() - calib_start_us; // us
        if (elapsed < 3 * 1000 * 1000) { // < 3s is likely invalid
                ESP_LOGW("CAL", "Measurement too short (%lld us). Aborted.", (long long)elapsed);
                calib_cancel();
                return;
        }
        uint32_t ms = (uint32_t)(elapsed / 1000);
        ESP_LOGI("CAL", "Calibration DONE: full_travel_ms=%" PRIu32, ms);
        full_travel_ms = ms;
        calib_save(ms);

        // Set position = 100 (fully open)
        pos_f = 100.0f;
        current_position.value = HOMEKIT_UINT8(100);
        position_state.value = HOMEKIT_UINT8(2);
        homekit_characteristic_notify(&current_position, current_position.value);
        homekit_characteristic_notify(&position_state, position_state.value);

        calib_state = CAL_IDLE;
        pix_state = PIX_IDLE;
}

// Implement recalibrate switch setter now that helpers exist
static void hk_recal_switch_set(homekit_value_t value) {
        bool on = (value.format == homekit_format_bool) ? value.bool_value : false;
        if (on) {
                // Toggle-like behavior: pressing while active cancels
                if (calib_state == CAL_IDLE) calib_enter();
                else calib_cancel();
        }
        // Momentary: auto-reset to OFF
        recalibrate_switch.value = HOMEKIT_BOOL(false);
        homekit_characteristic_notify(&recalibrate_switch, recalibrate_switch.value);
}

// -----------------------------------------------------------------------------
// Buttons
// -----------------------------------------------------------------------------

static void btn_open_callback(button_event_t event, void *context) {
        if (event != button_event_single_press) return;
        if (calib_state == CAL_ARMED) { calib_start_run(); return; }
        if (calib_state == CAL_RUNNING) return; // ignore targets during calibration
        start_move_open();
        (void)context;
}

static void btn_close_callback(button_event_t event, void *context) {
        if (event != button_event_single_press) return;
        if (calib_state != CAL_IDLE) return; // ignore during calibration
        start_move_close();
        (void)context;
}

static void btn_stop_callback(button_event_t event, void *context) {
        switch (event) {
        case button_event_single_press:
                if (calib_state == CAL_RUNNING) { calib_finish_on_stop(); break; }
                hold_position_set(HOMEKIT_BOOL(true));
                break;
        case button_event_double_press:
                if (calib_state != CAL_IDLE) break; // ignore during calibration
                ESP_LOGI("BTN", "STOP double-click -> move to %d%%", MID_POSITION);
                start_move_mid();
                break;
        case button_event_long_press:
                if (calib_state == CAL_IDLE) calib_enter();
                else calib_cancel();
                break;
        default:
                break;
        }
        (void)context;
}

static void buttons_init(void) {
        button_active_level_t active_level = CONFIG_BUTTON_ACTIVE_LEVEL ? button_active_high : button_active_low;

        button_config_t cfg_single = button_config_default(active_level);
        button_config_t cfg_stop = button_config_default(active_level);

        cfg_stop.long_press_time = 3000; // ms
        cfg_stop.max_repeat_presses = 2; // enable double press detection

        if (button_create(BTN_OPEN_GPIO, cfg_single, btn_open_callback, NULL)) {
                ESP_LOGE("BTN", "Failed to init OPEN button");
        }
        if (button_create(BTN_CLOSE_GPIO, cfg_single, btn_close_callback, NULL)) {
                ESP_LOGE("BTN", "Failed to init CLOSE button");
        }
        if (button_create(BTN_STOP_GPIO, cfg_stop, btn_stop_callback, NULL)) {
                ESP_LOGE("BTN", "Failed to init STOP button");
        }
}

// -----------------------------------------------------------------------------
// Accessory & config
// -----------------------------------------------------------------------------
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"
homekit_accessory_t *accessories[] = {
        HOMEKIT_ACCESSORY(.id = 1,
                          .category = homekit_accessory_category_window_coverings,
                          .services = (homekit_service_t*[]) {
                // Accessory Information
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics = (homekit_characteristic_t*[]) {
                        &name, &manufacturer, &serial, &model, &revision,
                        HOMEKIT_CHARACTERISTIC(IDENTIFY, accessory_identify),
                        NULL
                }),
                // Window Covering (primary)
                HOMEKIT_SERVICE(WINDOW_COVERING, .primary = true, .characteristics = (homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Sunshade"),
                        &current_position,
                        HOMEKIT_CHARACTERISTIC(TARGET_POSITION, 0, .setter = target_position_set),
                        &position_state,
                        &obstruction_detected,
                        HOMEKIT_CHARACTERISTIC(HOLD_POSITION, false, .setter = hold_position_set),
                        &ota_trigger,
                        NULL
                }),
                // Optional: Recalibrate switch (momentary)
                HOMEKIT_SERVICE(SWITCH, .characteristics = (homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Recalibrate"),
                        &recalibrate_switch,
                        NULL
                }),
                NULL
        }
                          ),
        NULL
};
#pragma GCC diagnostic pop

homekit_server_config_t config = {
        .accessories = accessories,
        .password = CONFIG_ESP_SETUP_CODE,
        .setupId  = CONFIG_ESP_SETUP_ID,
};

// -----------------------------------------------------------------------------
// Wi-Fi/HomeKit bootstrap
// -----------------------------------------------------------------------------
static void on_wifi_ready(void) {
        static bool started = false;
        if (started) return;
        ESP_LOGI("INFO", "Starting HomeKit server…");
        homekit_server_init(&config);
        started = true;
        // From now on, use idle color unless moving
        if (calib_state == CAL_IDLE) pix_state = PIX_IDLE;
}

// -----------------------------------------------------------------------------
// App main
// -----------------------------------------------------------------------------
void app_main(void) {
        // NVS init (for calibration storage)
        ESP_ERROR_CHECK(nvs_flash_init());

        ESP_ERROR_CHECK(lifecycle_nvs_init());
        lifecycle_log_post_reset_state("INFO");
        ESP_ERROR_CHECK(lifecycle_configure_homekit(&revision, &ota_trigger, "INFO"));

        // GPIOs
        gpio_init_led();
        gpio_init_motor();

        // NeoPixel
        neopixel_init();
        pix_state = PIX_WIFI_WAIT;

        // Buttons
        buttons_init();

        // Load calibration
        uint32_t nvs_ms = 0;
        if (calib_load(&nvs_ms) == ESP_OK) {
                full_travel_ms = nvs_ms;
                ESP_LOGI("CAL", "Loaded full_travel_ms from NVS: %" PRIu32 " ms", full_travel_ms);
        } else {
                ESP_LOGW("CAL", "No calibration found; using fallback %" PRIu32 " ms", full_travel_ms);
        }

        // Start Wi-Fi
        esp_err_t wifi_err = wifi_start(on_wifi_ready);
        if (wifi_err == ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW("WIFI", "WiFi configuration not found; provisioning required");
        } else if (wifi_err != ESP_OK) {
                ESP_LOGE("WIFI", "Failed to start WiFi: %s", esp_err_to_name(wifi_err));
        }
}
