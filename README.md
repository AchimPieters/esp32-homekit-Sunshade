# HomeKit Sunshade – ESP32-WROOM-32D

[![Build](https://github.com/AchimPieters/esp32-homekit-Sunshade/actions/workflows/build.yml/badge.svg)](https://github.com/AchimPieters/esp32-homekit-Sunshade/actions/workflows/build.yml)

A complete **Apple HomeKit Window Covering** accessory for the ESP32-WROOM-32D built with ESP-IDF v5.  
It replaces a standard UP/DOWN/STOP switch and adds HomeKit control, capacitive touch buttons, time-based position tracking, calibration, and automatic power-loss recovery.

---

## Table of Contents

1. [Feature overview](#1-feature-overview)
2. [Hardware](#2-hardware)
3. [GPIO pinout](#3-gpio-pinout)
4. [Build and flash](#4-build-and-flash)
5. [First-time setup](#5-first-time-setup)
6. [Calibration procedure](#6-calibration-procedure)
7. [Power-loss recovery](#7-power-loss-recovery)
8. [HomeKit position control](#8-homekit-position-control)
9. [Touch buttons](#9-touch-buttons)
10. [Physical button](#10-physical-button)
11. [Identify LED](#11-identify-led)
12. [OTA updates](#12-ota-updates)
13. [Wind speed sensor (optional)](#13-wind-speed-sensor-optional)
14. [Rain sensor (optional)](#14-rain-sensor-optional)
- [Light sensor (optional)](#light-sensor-optional--bh1750)
- [Temperature & humidity sensor (optional)](#temperature--humidity-sensor-optional--sht3x)
15. [Sunrise / sunset automations](#15-sunrise--sunset-automations)
16. [Menuconfig reference](#16-menuconfig-reference)
17. [NVS storage layout](#17-nvs-storage-layout)
18. [How position tracking works](#18-how-position-tracking-works)
19. [Troubleshooting](#19-troubleshooting)
20. [Requirements](#20-requirements)

---

## 1. Feature overview

| Feature | Details |
|---------|---------|
| **HomeKit service** | `WINDOW_COVERING` – current position, target position, position state, hold position |
| **Accessory category** | Window Covering (tile icon in the Home app) |
| **Relay outputs** | GPIO16 (OPEN/UP), GPIO17 (CLOSE/DOWN); software interlock, configurable active level, direction-reversal dead time |
| **Touch buttons** | TTP223 capacitive modules: UP (GPIO32), STOP (GPIO33), DOWN (GPIO27) |
| **Physical button** | Push button on GPIO25: single/double/long press |
| **Identify LED** | GPIO23 – blinks on HomeKit Identify, not visible as a service |
| **Calibration** | Measures actual motor travel time; saved to NVS flash |
| **Power-loss recovery** | On boot: closes fully, then restores last HomeKit position |
| **Position tracking** | Time-based, 0–100 %, notified on change (max every 500 ms, HAP-compliant) |
| **OTA** | Firmware update via HomeKit custom characteristic or single button press |
| **Weather protection (optional)** | Auto-close on high wind (HWFS-1), rain (MH-RD) or bright sun (BH1750); restores the previous position when the condition clears |
| **Environmental sensors (optional)** | Temperature & humidity (SHT3x) and ambient light (BH1750) shown in the Home app as HomeKit sensor tiles |
| **Lifecycle Manager** | WiFi, NVS, factory reset, reboot counter via `esp32-lcm` |

---

## 2. Hardware

### Required components

| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32-WROOM-32D | 1 | Any ESP32 module with sufficient GPIOs |
| Relay module (2-channel) | 1 | One relay per direction; optoisolated recommended |
| Copper ring / touch pad | 3 | UP, STOP, DOWN – with shielded cable and ground shield |
| Push button (momentary) | 1 | NO type, wired GND → button → GPIO25 |
| LED + current-limiting resistor | 1 | Identify LED on GPIO23 |

### Schematic

![HomeKit Sunshade wiring diagram](Sunshade.jpg)

### Relay wiring

The two relay outputs switch the motor direction:

```
OPEN relay (GPIO16) ──► motor terminal A power
CLOSE relay (GPIO17) ──► motor terminal B power
GND (common) ──────────► motor common
```

> **Important:** Never wire OPEN and CLOSE to the same motor terminal simultaneously. The software interlock already prevents both relays from being energised at the same time, but double-check your wiring.

> **Relay active level:** The firmware drives the relays active-high by default (`ESP_RELAY_ACTIVE_LEVEL = 1`). Many common opto-isolated 2-channel modules are **active-low** (the relay turns on when the input pin is pulled LOW). If both relays click on at boot and the motor runs unexpectedly, set `ESP_RELAY_ACTIVE_LEVEL = 0` in menuconfig. Always test with the motor mechanically disconnected first.

> **Direction-reversal dead time:** AC tubular motors must not be switched straight from one direction to the other. Before energising the opposite direction the firmware holds both relays off for `ESP_RELAY_REVERSE_DELAY_MS` (default 500 ms). A hardware interlock (cross-wired NC contacts) is still recommended for mains-driven motors.

### Touch pad construction

Each capacitive button uses a **TTP223 module** between the electrode and the ESP32:

- A **copper ring or pad** as the sensing electrode, wired to the TTP223 SIG input
- A **TTP223 capacitive touch module** (momentary mode, active-high) providing a clean digital output
- A **shielded cable** (braid connected to GND) between the pad and the module to prevent false positives
- ESP32 reads the TTP223 digital output via a regular GPIO input — the internal touch peripheral is not used

---

## 3. GPIO pinout

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| **GPIO16** | OPEN / UP relay | Output | Active level configurable (default HIGH); motor runs UP |
| **GPIO17** | CLOSE / DOWN relay | Output | Active level configurable (default HIGH); motor runs DOWN |
| **GPIO23** | Identify LED | Output | Blinks on HomeKit Identify |
| **GPIO25** | Physical push button | Input | GND → button → GPIO25; active-low |
| **GPIO27** | TTP223 DOWN signal | Digital input | Active-high; close/lower sunshade |
| **GPIO32** | TTP223 UP signal | Digital input | Active-high; open/raise sunshade |
| **GPIO33** | TTP223 STOP signal | Digital input | Active-high; stop + calibration trigger |

### Optional sensor pins

Only used when the matching feature is enabled in `menuconfig`:

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| **GPIO34** | HWFS-1 wind sensor | ADC1 input | Via voltage divider; see [Wind sensor](#13-wind-speed-sensor-optional--hwfs-1) |
| **GPIO35** | MH-RD rain sensor | Digital input | DO pin, active-low; see [Rain sensor](#14-rain-sensor-optional--mh-rd) |
| **GPIO21** | Shared I²C SDA | I²C | BH1750 + SHT3x; see [Light](#light-sensor-optional--bh1750) / [Climate](#temperature--humidity-sensor-optional--sht3x) |
| **GPIO22** | Shared I²C SCL | I²C | BH1750 + SHT3x on the same bus |

> The BH1750 light sensor and the SHT3x temperature/humidity sensor share one I²C bus, so SDA/SCL are configured once (`I2C_MASTER_SDA_GPIO` / `I2C_MASTER_SCL_GPIO`).

All defaults are configurable via `idf.py menuconfig` → **StudioPieters**.

---

## 4. Build and flash

```bash
git clone https://github.com/AchimPieters/esp32-homekit-Sunshade.git
cd esp32-homekit-Sunshade

# Set target (once per checkout)
idf.py set-target esp32

# Optional: adjust GPIOs, travel time, HomeKit code
idf.py menuconfig

# Build, flash and monitor
idf.py build flash monitor
```

> If `idf.py set-target` fails with a CMake error, remove the stale build folder first:
> ```bash
> rm -rf build && idf.py set-target esp32
> ```

### Provision WiFi

On first boot, WiFi credentials are not stored. Use any ESP-IDF provisioning method (e.g. `esp-idf-prov` app on iOS/Android) to configure the SSID and password. The device will remember the credentials across reboots.

---

## 5. First-time setup

1. **Flash** the firmware.
2. **Provision WiFi** (see above).
3. **Pair** with HomeKit using the QR code or the setup code printed in `menuconfig` (default: `582-94-633`).
4. **Calibrate** the sunshade so the device learns how long your motor takes (see [Calibration procedure](#6-calibration-procedure)).
5. After calibration, **HomeKit position control** (0–100 %) is active.

---

## 6. Calibration procedure

Calibration teaches the device the actual travel time of your motor by measuring it directly. The result is stored in NVS flash and survives power loss.

> **Trigger:** Hold the **STOP touch pad** for **3 seconds**.  
> The LED starts blinking rapidly to confirm calibration mode has started.

### Phase 1 – close to home position

The motor runs **CLOSED** for `calibrated_time × 1.5`. This guarantees the sunshade reaches the physical end stop even if the stored time is slightly off. The motor stops itself at the end stop; the extra time is harmless.

LED blinks **fast** (100 ms on/off) during this phase.

### Phase 2 – open and measure

The motor starts running **OPEN** and a timer counts the elapsed time.  
LED blinks **slowly** (400 ms on/off).

The **Home app shows the sunshade opening in real time** during this phase — the position indicator moves from 0 % upward based on the previously stored travel time. Use this as a visual guide.

**Watch the sunshade. When it is fully open:**  
→ Press the **STOP touch pad once** (quick tap).

The elapsed time is saved to NVS as the calibrated travel time (`shade/cal_ms`).

### Calibration result

| Result | LED pattern | HomeKit |
|--------|-------------|---------|
| **Success** | 5 short flashes | Position set to 100 %; all positions 0–100 % available |
| **Aborted** (STOP < 2 s after phase 2 start) | 3 long flashes | Position reset to 0 % |
| **Timeout** (no STOP within 2 minutes) | 3 long flashes | Position reset to 0 % |

If calibration fails, repeat the procedure.

### Re-calibration

To update the travel time (e.g. after motor replacement), simply trigger calibration again. The new value overwrites the stored one.

---

## 7. Power-loss recovery

After every power loss the device does not know where the sunshade is.  
If the device **has been calibrated**, it performs an automatic **homing sequence** on boot:

```
Boot
 └─ Calibrated?
     ├─ Yes → Homing task (background)
     │         1. Run motor CLOSED for (cal_ms × 1.5)
     │            Motor stops at physical end stop → position = 0 %
     │         2. Wait 500 ms
     │         3. Load last saved target position from NVS
     │         4. Move to that position
     │
     └─ No  → Start normally; show warning in log
               (hold STOP touch 3 s to calibrate)
```

The homing task runs **in the background** so WiFi and HomeKit initialise in parallel. Commands from HomeKit or the touch buttons are blocked while homing is in progress.

### What is "last saved position"?

Every time you command the sunshade to a new position (via HomeKit or touch buttons), the **target** position is saved to NVS (`shade/last_pos`). After a power loss, the device restores this value.

**Example:**
1. HomeKit commands 70 % → saved to NVS.
2. Power fails while moving from 0 % to 70 % (sunshade is at ~40 %).
3. On reboot: close fully → position = 0 % → move to 70 %.

---

## 8. HomeKit position control

Once calibrated, the **Sun Screen** tile in the Home app supports:

| Control | Result |
|---------|--------|
| Drag slider to 0 % | Close sunshade fully |
| Drag slider to 100 % | Open sunshade fully |
| Drag slider to 20–99 % | Move to that exact position |
| Tap tile (toggle) | Fully open or fully close |
| **"Hey Siri, stop the sunshade"** | Stop at current position (`HOLD_POSITION`) |

### Position state

The Home app shows the current movement direction:

| HomeKit value | Meaning |
|---------------|---------|
| Decreasing | Closing |
| Increasing | Opening |
| Stopped | Motor off |

### End-stop behaviour for 0 % and 100 %

When the target is **0 %** or **100 %**, the motor runs for the full calibrated time. The motor's own mechanical end stop provides the final accuracy. HomeKit is notified of 0 % / 100 % when the timed run completes.

### Intermediate positions (1–99 %)

The motor runs for a calculated fraction of the calibrated time and stops via software. Accuracy depends on how close `cal_ms` is to the real travel time.

---

## 9. Touch buttons

Three **TTP223 capacitive touch modules** provide digital (HIGH/LOW) signals to the ESP32. The ESP32 reads these as regular GPIO inputs — it does not use its internal touch-sensing peripheral.

| Pad | GPIO | Action |
|-----|------|--------|
| UP | GPIO32 | Open sunshade to 100 % |
| STOP | GPIO33 | Stop immediately (or confirm calibration open) |
| DOWN | GPIO27 | Close sunshade to 0 % |

### TTP223 module configuration

Configure each TTP223 module as:
- **Momentary mode** (not toggle)
- **Active-high output** (default for most modules)
- **VCC = 3.3 V**, **GND = GND**

### Hold STOP for calibration

| Duration | Action |
|----------|--------|
| Quick tap (< 3 s) | Stop motor or confirm calibration open |
| Hold ≥ 3 s | Start calibration procedure |

### Debounce

Inputs are polled every `ESP_TTP_POLL_MS` (default 25 ms) and debounced over `ESP_TTP_DEBOUNCE_MS` (default 60 ms). Both values are configurable via `idf.py menuconfig`.

---

## 10. Physical button

Wire: **GND → button → GPIO25** (active-low, internal pull-up).

| Press type | Action |
|------------|--------|
| **Single press** | Request OTA firmware update and reboot |
| **Double press** | Reset HomeKit pairing + restart |
| **Long press (≥ 1 s)** | Factory reset (WiFi + HomeKit) + reboot |

The relays are turned off before any reset or reboot.

---

## 11. Identify LED

GPIO23, active-high output.

When HomeKit sends an **Identify** request (e.g. when adding the accessory), the LED blinks 3 × 2 pulses. The LED is **not** exposed as a HomeKit Lightbulb service.

---

## 12. OTA updates

Two ways to trigger a firmware update:

1. **Single press** on the physical button → `lifecycle_request_update_and_reboot()`
2. **HomeKit** → toggle the custom OTA characteristic in the Window Covering service

The Lifecycle Manager handles the update process and version tracking automatically.

---

## 13. Wind speed sensor (optional) — HWFS-1

The **HWFS-1** is a passive cup anemometer with a 0–4 V analog output. It can automatically close the sunshade when wind exceeds a configurable threshold.

Enable via `idf.py menuconfig` → **StudioPieters** → **Wind Speed Sensor**.

### Sensor characteristics

| Property | Value |
|----------|-------|
| Model | HWFS-1 (0–3.3 V version) |
| Power | **Passive — no external VCC needed** (self-powered by rotation) |
| Output | 0–3.3 V analog, DC generator |
| Full scale | 3.3 V = **14 m/s** (manufacturer stated) |
| Conversion | `wind speed (m/s) = V_out × (14 / 3.3) ≈ V_out × 4.24` |
| Resting offset | ~33 mV at 0 m/s (not true 0 V) |
| Accuracy | ±10 % (manufacturer); nonlinear at very low RPM |
| Cable | 2 conductors: signal (black + stripe) + GND (solid black) |

> The HWFS-1 uses a small DC permanent-magnet generator. At very low RPM, cogging torque and brush friction cause nonlinearity. For sunshade protection (thresholds at 5–8 m/s), the sensor is well within its operating range. Calibrate against a reference instrument and adjust `WIND_SENSOR_MAX_SPEED_DS` if needed.

### Wiring

The sensor output reaches 4 V which exceeds the ESP32's 3.3 V ADC maximum. A voltage divider is required. Although the sensor output (3.3 V max) matches the ESP32 supply voltage, the ADC is only accurate up to **2450 mV** with `ADC_ATTEN_DB_11`. Use **R1 = 10 kΩ + R2 = 22 kΩ** to stay within that range.

```
HWFS-1 signal wire (black+stripe)
         │
         ├── R1 (10 kΩ) ──┬── R2 (22 kΩ) ──► GND
         │                │
         │          (optional RC filter:
         │           10 kΩ + 10 µF to GND,
         │           τ ≈ 100 ms, reduces ripple)
         │                │
         └────────────────┴──► ESP32 GPIO34 (ADC)

HWFS-1 ground wire (solid black) ──► GND
```

At 3.3 V sensor output (14 m/s): V_ADC = 3.3 × 22 / (10 + 22) = **2.27 V** ✓ (within 2450 mV limit)  
At 8.0 m/s: V_sensor = 8 / 14 × 3.3 = 1.89 V → V_ADC = 1.89 × 22/32 = **1.30 V** ✓

> No VCC connection is needed. The HWFS-1 generates its own signal voltage from wind rotation.

> Only ADC1 GPIOs (GPIO32–39) are supported. ADC2 conflicts with WiFi.

### Conversion formula

The firmware uses a linear mapping:

```
speed (m/s) = (V_ADC / V_ADC_fullscale) × MAX_SPEED_DS / 10
```

With the defaults (R1=10k/R2=22k, `WIND_SENSOR_ADC_FULL_SCALE_MV = 2270`, `WIND_SENSOR_MAX_SPEED_DS = 140`):

```
speed (m/s) = V_out × (14 / 3.3) ≈ V_out × 4.24
```

The resting offset (~23 mV at the ADC pin after divider) is masked — readings at or below 30 mV are reported as 0 m/s.

### ADC noise and DC generator ripple

The HWFS-1's commutator generates voltage ripple. Two mitigation layers are implemented:

- **Software:** 16 ADC samples are averaged per reading (5 ms between samples).
- **Hardware (recommended):** Place a 10 kΩ + 10 µF RC low-pass filter (τ ≈ 100 ms) on the signal wire before the ADC pin.

### Behaviour

| Condition | Action |
|-----------|--------|
| Wind ≥ close threshold (default **8.0 m/s**) | Sunshade closes; previous target position is saved |
| Wind < reopen threshold (default **5.0 m/s**) | Sunshade restores to the saved position |

The hysteresis gap (3 m/s by default) prevents rapid cycling when wind hovers near the threshold.

### Field calibration procedure

1. Flash with `WIND_SENSOR_ENABLE = y` and default settings.
2. At startup the log prints the effective calibration factor: `HWFS-1 ready: ... cal factor V×2.5 m/s`.
3. Run `idf.py monitor` and observe the `WIND:` log lines: `1.2 m/s (avg_raw=XXX, 120 mV)`.
4. Hold a calibrated reference anemometer next to the HWFS-1 in steady wind.
5. Compare the readings. If they differ significantly, calculate the correct factor and adjust `WIND_SENSOR_MAX_SPEED_DS` until the readings match.
6. Example: if the reference shows 6.0 m/s but firmware shows 3.5 m/s, increase `MAX_SPEED_DS` by the ratio (6.0/3.5 × 103 ≈ 177).

---

## 14. Rain sensor (optional) — MH-RD

A digital rain sensor module can automatically close the sunshade when it starts raining and restore the previous position when it stops.

Enable via `idf.py menuconfig` → **StudioPieters** → **Rain Sensor**.

### Sensor characteristics

| Property | Value |
|----------|-------|
| Module | MH-RD (LM393 comparator) |
| Power | VCC = 3.3 V, GND = GND |
| Output used | **DO** (digital output) — active-low when rain detected |
| Output unused | AO (analog output) — not connected |
| Sensitivity | Adjustable via onboard potentiometer |
| Lifespan outdoors | Mount in sheltered position; sensing pad corrodes when permanently exposed |

### Wiring

```
MH-RD module
    VCC ──► 3.3 V
    GND ──► GND
    DO  ──► ESP32 GPIO35
    AO     (not connected)
```

> Only the **DO** pin is used. The **AO** (analog) pin is not connected.

### Behaviour

| Condition | Action |
|-----------|--------|
| Rain detected (DO stable for **2 s**) | Sunshade closes; previous target position is saved |
| Rain stopped (DO stable for **2 s**) | Sunshade restores to the saved position |

The 2-second debounce prevents false triggers from brief drizzle or sensor noise. Adjust `RAIN_SENSOR_DEBOUNCE_MS` in menuconfig if needed.

### Menuconfig keys

| Config key | Default | Description |
|------------|---------|-------------|
| `RAIN_SENSOR_ENABLE` | n | Enable MH-RD rain sensor |
| `RAIN_SENSOR_GPIO` | 35 | GPIO connected to the DO pin |
| `RAIN_SENSOR_ACTIVE_LEVEL` | 0 | 0 = active-low (default); 1 = active-high |
| `RAIN_SENSOR_POLL_MS` | 500 | Poll interval in ms |
| `RAIN_SENSOR_DEBOUNCE_MS` | 2000 | Stable time before acting on state change (ms) |

### Installation tip

Mount the sensing pad under a sheltered overhang so only falling rain reaches it — not condensation, irrigation spray, or splashing. This significantly extends pad lifespan and reduces false triggers.

---

## Light sensor (optional) — BH1750

A **BH1750** (GY-302) digital ambient-light sensor can automatically lower the sunshade in bright sun and restore the previous position once it clouds over. The illuminance is also published as a **HomeKit Light sensor** tile in the Home app.

Enable via `idf.py menuconfig` → **StudioPieters** → **Light Sensor**. It shares the I²C bus with the SHT3x climate sensor, so SDA/SCL are set once under the shared I²C keys.

### Sensor characteristics

| Property | Value |
|----------|-------|
| Module | BH1750 / GY-302 (I²C) |
| Power | VCC = 3.3 V, GND = GND |
| Bus | I²C — SDA (default GPIO21), SCL (default GPIO22) |
| Address | `0x23` (ADDR → GND, default) or `0x5C` (ADDR → 3.3 V) |
| Mode | Continuous high-resolution (1 lux, ~120 ms/sample) |
| Conversion | `lux = raw_count / 1.2` |

### Wiring

```
BH1750 (GY-302)
    VCC ──► 3.3 V
    GND ──► GND
    SDA ──► ESP32 GPIO21
    SCL ──► ESP32 GPIO22
    ADDR   (GND = 0x23, default) or (3.3 V = 0x5C)
```

> GY-302 breakouts include onboard I²C pull-ups. The firmware also enables the ESP32 internal pull-ups as a safety net.

### Behaviour

| Condition | Action |
|-----------|--------|
| Light ≥ close threshold (default **40000 lux**) | Sunshade closes; previous target position is saved |
| Light < reopen threshold (default **20000 lux**) | Sunshade restores to the saved position |

The hysteresis gap (20000 lux by default) prevents rapid cycling when a thin cloud passes. Bright direct sun is roughly 30000–100000 lux; an overcast day is a few thousand lux.

### Menuconfig keys

| Config key | Default | Description |
|------------|---------|-------------|
| `LUX_SENSOR_ENABLE` | n | Enable BH1750 light sensor |
| `LUX_SENSOR_I2C_ADDR` | 0x23 | 7-bit I²C address (0x23 or 0x5C) |
| `LUX_SENSOR_CLOSE_THRESHOLD_LUX` | 40000 | Auto-close threshold in lux |
| `LUX_SENSOR_REOPEN_THRESHOLD_LUX` | 20000 | Reopen hysteresis threshold in lux |
| `LUX_SENSOR_POLL_MS` | 5000 | Poll interval in ms |
| `I2C_MASTER_SDA_GPIO` | 21 | Shared I²C SDA GPIO (BH1750 + SHT3x) |
| `I2C_MASTER_SCL_GPIO` | 22 | Shared I²C SCL GPIO (BH1750 + SHT3x) |

> Wind, rain and light protection share the same override mechanism: whichever triggers saves the current HomeKit target and restores it when the condition clears.

---

## Temperature & humidity sensor (optional) — SHT3x

A **Sensirion SHT3x** (SHT30/31/35) measures temperature and relative humidity over I²C. Both readings are published as **HomeKit Temperature and Humidity sensor** tiles in the Home app. They are display-only — they do **not** move the sunshade.

Enable via `idf.py menuconfig` → **StudioPieters** → **Temperature & Humidity Sensor**.

### Sensor characteristics

| Property | Value |
|----------|-------|
| Module | Sensirion SHT30 / SHT31 / SHT35 (I²C) |
| Power | VCC = 3.3 V, GND = GND |
| Bus | Shared I²C — SDA (default GPIO21), SCL (default GPIO22) |
| Address | `0x44` (ADDR → GND, default) or `0x45` (ADDR → 3.3 V) |
| Mode | Single-shot, high repeatability, clock stretching disabled |
| Range | −40…+125 °C, 0…100 %RH; each sample is CRC-8 validated |

### Wiring

```
SHT3x
    VCC ──► 3.3 V
    GND ──► GND
    SDA ──► ESP32 GPIO21   (shared with BH1750)
    SCL ──► ESP32 GPIO22   (shared with BH1750)
    ADDR   (GND = 0x44, default) or (3.3 V = 0x45)
```

> The SHT3x and BH1750 live on the same I²C bus. Wire both SDA lines together and both SCL lines together; the firmware addresses each device separately (0x44 vs 0x23).

### Menuconfig keys

| Config key | Default | Description |
|------------|---------|-------------|
| `TEMP_SENSOR_ENABLE` | n | Enable SHT3x temperature & humidity sensor |
| `TEMP_SENSOR_I2C_ADDR` | 0x44 | 7-bit I²C address (0x44 or 0x45) |
| `TEMP_SENSOR_POLL_MS` | 10000 | Poll interval in ms |
| `I2C_MASTER_SDA_GPIO` | 21 | Shared I²C SDA GPIO (BH1750 + SHT3x) |
| `I2C_MASTER_SCL_GPIO` | 22 | Shared I²C SCL GPIO (BH1750 + SHT3x) |

---

## 15. Sunrise / sunset automations

No firmware changes are needed. The Home app has built-in sunrise/sunset automation that uses your Home Hub's GPS location and local timezone — more accurate than any on-device calculation.

### Requirements

- A **Home Hub** running continuously: HomePod mini, HomePod, or Apple TV 4K
- **Location services** enabled for the Home app on the Home Hub device

Without a Home Hub, time-based automations do not run.

### Setting up a sunrise automation (open at sunrise)

1. Open the **Home** app → tap the **Automations** tab.
2. Tap **+** → **A time of day occurs**.
3. Select **Sunrise**. Optionally add an offset: tap **Sunrise** again and choose e.g. **30 minutes after sunrise**.
4. Tap **Next** → select the **Sun Screen** tile.
5. Tap **Next** → drag the position slider to the desired open position (e.g. 80 %).
6. Tap **Done**.

### Setting up a sunset automation (close at sunset)

Follow the same steps but select **Sunset** and set the position to **0 %**.

### Recommended automation set

| When | Position | Purpose |
|------|----------|---------|
| Sunrise | 80 % | Open partially in the morning |
| Sunrise + 2 h | 100 % | Fully open once sun is higher |
| Sunset − 30 min | 0 % | Close before it gets dark |

### Adding conditions

Tap **Add Condition** on the automation screen to restrict when an automation runs:

| Condition | Example use |
|-----------|-------------|
| **People** | Only open when someone is home |
| **Time range** | Only run between April and September |
| **Accessory state** | Only open if a door/window sensor is closed |

### Combining with wind and rain protection

The wind and rain tasks run in firmware and override HomeKit commands when triggered. If the Home app opens the sunshade at sunrise but wind exceeds the threshold shortly after, the firmware closes it automatically. When wind drops, it restores the position the Home app set — not the sunrise position. This is correct behaviour: the last HomeKit-commanded position is always the restore target.

---

## 16. Menuconfig reference  

Open with `idf.py menuconfig` → **StudioPieters**.

| Config key | Default | Description |
|------------|---------|-------------|
| `ESP_LED_GPIO` | 23 | GPIO for the identify LED |
| `ESP_BUTTON_GPIO` | 25 | GPIO for the physical push button |
| `ESP_RELAY_OPEN_GPIO` | 16 | GPIO for the OPEN/UP relay |
| `ESP_RELAY_CLOSE_GPIO` | 17 | GPIO for the CLOSE/DOWN relay |
| `ESP_RELAY_ACTIVE_LEVEL` | 1 | Level that energises a relay (1 = active-high, 0 = active-low boards) |
| `ESP_RELAY_REVERSE_DELAY_MS` | 500 | Dead time both relays stay off before reversing direction |
| `ESP_TTP_UP_GPIO` | 32 | TTP223 UP module signal GPIO |
| `ESP_TTP_STOP_GPIO` | 33 | TTP223 STOP module signal GPIO |
| `ESP_TTP_DOWN_GPIO` | 27 | TTP223 DOWN module signal GPIO |
| `ESP_TTP_ACTIVE_LEVEL` | 1 | Active level for TTP223 output (1 = active-high) |
| `ESP_TTP_POLL_MS` | 25 | GPIO poll interval in ms (10–200) |
| `ESP_TTP_DEBOUNCE_MS` | 60 | Debounce window in ms (10–500) |
| `SUNSHADE_FULL_TRAVEL_TIME_MS` | 20000 | Default travel time in ms (used before calibration) |
| `ESP_SETUP_CODE` | 582-94-633 | HomeKit pairing code |
| `ESP_SETUP_ID` | 7MX2 | HomeKit setup ID |

> Changing `ESP_SETUP_CODE` or `ESP_SETUP_ID` requires a new QR code.

**Wind Speed Sensor** (`WIND_SENSOR_ENABLE = y`):

| Config key | Default | Description |
|------------|---------|-------------|
| `WIND_SENSOR_ENABLE` | n | Enable HWFS-1 wind speed sensor |
| `WIND_SENSOR_ADC_GPIO` | 34 | ADC1 GPIO for signal wire (after voltage divider) |
| `WIND_SENSOR_ADC_FULL_SCALE_MV` | **2270** | ADC voltage (mV) at sensor full-scale (3.3 V → 2270 mV with R1=10k/R2=22k) |
| `WIND_SENSOR_MAX_SPEED_DS` | **140** | Full-scale wind speed in dm/s (140 = 14.0 m/s, HWFS-1 0–3.3 V version) |
| `WIND_SENSOR_CLOSE_THRESHOLD_DS` | 80 | Close threshold in dm/s (80 = 8.0 m/s, Beaufort 5) |
| `WIND_SENSOR_REOPEN_THRESHOLD_DS` | 50 | Reopen hysteresis in dm/s (50 = 5.0 m/s) |
| `WIND_SENSOR_POLL_MS` | 2000 | ADC sample interval in ms |

---

## 17. NVS storage layout

| Namespace | Key | Type | Description |
|-----------|-----|------|-------------|
| `shade` | `cal_done` | u8 | `1` = device has been calibrated |
| `shade` | `cal_ms` | u32 | Measured full travel time in milliseconds |
| `shade` | `last_pos` | u8 | Last HomeKit target position (0–100) |

The Lifecycle Manager uses a separate namespace for WiFi credentials and reboot counters.

---

## 18. How position tracking works

The device uses **time-based position estimation** because there are no limit switches or encoders.

```
position_delta_per_tick = 100 % × 500 ms / cal_ms
```

Every 500 ms the movement task adds or subtracts this delta from the current position. HomeKit is only notified when the integer position value actually changes, keeping notification traffic within the HAP-recommended rate. When the target is reached, the relay is switched off and HomeKit is notified unconditionally.

### Accuracy

- Accuracy depends directly on how well `cal_ms` matches your motor's actual travel time.
- The motor's mechanical end stop corrects accumulated error every time the sunshade reaches 0 % or 100 %.
- For intermediate positions, expect ±2–5 % error over many cycles without recalibration.

### Direction change mid-travel

If a new command arrives while the motor is running (e.g. stop at 60 % then move to 30 %), the task re-syncs its floating-point accumulator from `s_cur_pos` and immediately reflects the direction change. The relay interlock ensures the closing relay is always off before the opening relay is energised, and vice versa.

---

## 19. Troubleshooting

### Sunshade does not respond to touch

1. Check the log for `TTP223 inputs:` at boot. The GPIO numbers and active level are printed there.
2. Verify that each TTP223 module is configured for **momentary mode** and **active-high output**.
3. Confirm VCC is 3.3 V and GND is connected.
4. Reduce `ESP_TTP_DEBOUNCE_MS` in menuconfig if the module output is stable but not registering (default 60 ms).

### Calibration aborts immediately

- STOP touch pad pressed within 2 seconds of phase 2 start → counts as premature.
- Repeat calibration and wait until the sunshade is fully open before tapping STOP.

### Position is wrong after several cycles

Recalibrate. Measure the actual travel time with a stopwatch; if it differs significantly from `cal_ms`, the motor speed may vary with load or temperature.

### HomeKit shows wrong position after power loss

- Confirm the device is calibrated (`shade/cal_done = 1` in NVS).
- The homing sequence is logged: `[SUNSHADE] Homing: closing fully...`. If not seen, calibration data may be missing.
- If NVS is corrupted, perform a factory reset (long press button) and recalibrate.

### Relay clicks but motor does not run

- Check relay module power supply (some modules need 5 V even if the coil is driven by 3.3 V logic).
- Verify the NO/COM wiring on the relay module.

### WiFi will not connect

- Double press the physical button to reset HomeKit pairing, then re-provision WiFi.
- Long press the physical button for a full factory reset.

### Wind sensor reads 0 or incorrect speed

1. Confirm `WIND_SENSOR_ENABLE = y` in menuconfig.
2. Check `WIND: Wind sensor ready:` in the boot log. If absent, the task failed to start.
3. Verify the voltage divider: measure V_ADC with a multimeter while blowing on the sensor. It should increase from ~0 V (no wind) toward the configured `WIND_SENSOR_ADC_FULL_SCALE_MV`.
4. Confirm the GPIO is on ADC1 (GPIO32–39). The log prints an error if ADC2 is used.
5. Increase `WIND_SENSOR_POLL_MS` if you see ADC read errors in the log.

### Sunshade does not close on high wind

- Check the serial log for `WIND: Wind X.X m/s` messages to confirm readings.
- Lower `WIND_SENSOR_CLOSE_THRESHOLD_DS` in menuconfig.
- The HWFS-1 is self-powered — no supply voltage is needed. Verify only signal and GND are connected.

### Rain sensor does not trigger

1. Confirm `RAIN_SENSOR_ENABLE = y` in menuconfig.
2. Check `RAIN: MH-RD ready:` in the boot log. If absent, the task failed to start.
3. Measure the DO pin with a multimeter while wetting the sensing pad. It should switch between ~0 V (rain) and ~3.3 V (dry) for active-low mode.
4. Adjust the onboard potentiometer clockwise to increase sensitivity.
5. If the sunshade does not close after rain starts, increase `RAIN_SENSOR_DEBOUNCE_MS` to filter noise, or decrease it if the response is too slow.

### Sunshade closes unexpectedly (false rain trigger)

- Mount the sensing pad in a sheltered position away from condensation and irrigation spray.
- Increase `RAIN_SENSOR_DEBOUNCE_MS` (default 2000 ms) to require longer stable detection.
- Turn the potentiometer counter-clockwise to reduce sensitivity.

### `idf.py set-target` CMake error

```bash
rm -rf build
idf.py set-target esp32
```

---

## Continuous integration & tests

Every push and pull request runs two GitHub Actions jobs (`.github/workflows/build.yml`):

1. **Host unit tests** — compile and run `test/test_sunshade_logic.c` against the pure logic in `main/sunshade_logic.h` (relay polarity, position math, sensor conversions, hysteresis). No hardware or ESP-IDF needed.
2. **ESP-IDF build** — builds the full firmware for `esp32` on ESP-IDF v5.3.2 and v5.4.1. Runs only after the unit tests pass.

Run the unit tests locally:

```bash
cc -std=c11 -Wall -Wextra -Werror -I main test/test_sunshade_logic.c -o /tmp/sunshade_test && /tmp/sunshade_test
```

---

## 20. Requirements

| Component | Version |
|-----------|---------|
| ESP-IDF | `>=5.0, <7.0` |
| `achimpieters/esp32-homekit` | `>=3.0.0` |
| `achimpieters/esp32-button` | `>=1.2.3` |
| `espressif/mdns` | `>=1.8.0` |

---

*Copyright 2026 Achim Pieters | StudioPieters® – [studiopieters.nl](https://www.studiopieters.nl)*
