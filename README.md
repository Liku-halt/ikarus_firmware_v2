# Ikarus Firmware v2

A lightweight, firmware-driven quadcopter flight controller built on **ESP-IDF v5.x** for the **ESP32-C3** (single-core RISC-V MCU). It implements a full flight control stack — IMU sensing, complementary-filter attitude estimation, a cascaded PID loop, and ESP-NOW-based RC/telemetry — running on a hard-timed 1 kHz control loop, entirely in fixed-point (Q16.16) arithmetic.

> ⚠️ **Status: work in progress.** This is an active hobby/research project, not a finished or flight-proven product. Expect rough edges, dead code paths, and breaking changes.

## Gallery

<table>
<tr>
<td width="50%">

**Custom flight controller PCB**
![Custom flight controller PCB](WhatsApp%20Image%202026-09-08%20at%2012.36.50.jpeg)

</td>
<td width="50%">

**KiCad routing**
![KiCad PCB routing](WhatsApp%20Image%202026-09-08%20at%2012.37.42.jpeg)

</td>
</tr>
<tr>
<td width="50%">

**Frame CAD model**
![Frame CAD render](WhatsApp%20Image%202026-09-08%20at%2012.37.43.jpeg)

</td>
<td width="50%">

**Assembled quad, bench test**
![Assembled quadcopter on bench](WhatsApp%20Image%202026-09-08%20at%2012.37.04.jpeg)

</td>
</tr>
</table>

**Flight test clip**

<video src="WhatsApp%20Video%202026-09-08%20at%2012.37.04.mp4" controls width="600">
Your browser/viewer doesn't support inline video — <a href="WhatsApp%20Video%202026-09-08%20at%2012.37.04.mp4">watch it here</a> instead.
</video>

## Overview

Ikarus is a from-scratch flight controller stack, written without an RTOS abstraction layer beyond FreeRTOS itself — no Betaflight/ArduPilot dependency. The goals of this rewrite (v2) are:

- A precisely-timed 1 ms control loop driven by a hardware general-purpose timer (`gptimer`), rather than a `vTaskDelay` loop
- Fixed-point (Q16.16) math throughout the hot path (attitude estimation, PID, mixer) instead of floats, to keep loop timing deterministic
- A custom lookup-table-based `atan2` implementation (`atan_lut.c/h`) for fast angle computation without a math-library dependency
- Direct MPU6050 access over I²C (no external sensor-fusion library)
- ESP-NOW for low-latency RC command input and telemetry (no Wi-Fi AP/STA pairing overhead)

### Known limitations

- The 1 ms loop currently experiences jitter because ESP-NOW's driver callback is inherently blocking on this single-core MCU (ESP32-C3), which occasionally delays the control loop.
- A future revision is planned on a more capable MCU (dual-core / higher clock) to tighten loop timing to <0.5 ms and to make room for an onboard state-estimation stack aimed at swarm applications.

## Hardware

| Component | Details |
|---|---|
| MCU | ESP32-C3 (single-core RISC-V, ESP-IDF v5.x) |
| IMU | MPU6050 (I²C, accelerometer + gyroscope) |
| Motor output | 4× PWM channels via `LEDC` (10-bit resolution, 32 kHz) |
| Radio link | ESP-NOW (peer-to-peer, channel 4) |
| Telemetry / debug | UART0 |

### Pin mapping

| Signal | GPIO |
|---|---|
| I²C SDA | 6 |
| I²C SCL | 7 |
| Motor 1 (PWM) | 4 |
| Motor 2 (PWM) | 5 |
| Motor 3 (PWM) | 8 |
| Motor 4 (PWM) | 10 |
| UART TX | 1 |
| UART RX | 3 |

## Repository layout

```
.
├── CMakeLists.txt              # Top-level ESP-IDF project file
├── main/
│   ├── CMakeLists.txt
│   ├── ikarus_firmware_v2.c    # Application entry point, control loop, PID, IMU, ESP-NOW
│   ├── atan_lut.c / atan_lut.h # Fixed-point lookup-table atan2 implementation
├── sdkconfig                   # ESP-IDF project configuration (target: esp32c3)
├── flight_2_1.pdf              # Flight log / test notes
├── Part Studio 1 - ikaris_new555.stl   # Frame / mechanical CAD model
└── WhatsApp Image/Video *.jpeg/.mp4    # Build photos + flight test clip (see Gallery above)
```

## How it works

1. **Sensor acquisition** — `get_raw_mpu()` reads accelerometer and gyro data from the MPU6050 over I²C and converts it to Q16.16 fixed-point.
2. **Attitude estimation** — `bias_estim()` combines gyro-integrated angle with an accelerometer-derived angle (via the custom `angle_lut` atan2 approximation) and a slow bias correction term, functioning as a complementary filter.
3. **Error + PID** — `error_calculation()` computes the error between commanded and estimated attitude; `pid_calc()` runs a cascaded PID (outer angle loop → inner rate loop, all in fixed-point) with integrator clamping.
4. **Mixing + output** — the control task combines roll/pitch/yaw PID outputs with throttle into 4 motor commands, clamps them, and updates PWM duty via `pwm_update()`.
5. **Timing** — a hardware `gptimer` fires every 1 ms and sets a flag; the control task polls that flag and runs the loop above only when it's set, keeping the loop period as close to 1 kHz as the platform allows.
6. **Command input** — an ESP-NOW receive callback (`on_callback`) fills a `data_construct` struct (roll/pitch/yaw/thrust commands, live PID gains, and an arm/disarm flag) sent from a companion transmitter/controller.
7. **Arming logic** — on `arm`, throttle ramps up to a hover baseline if a fresh command was seen recently, otherwise it ramps down as a failsafe; on `disarm`, all motors are cut immediately.

## Building & flashing

This project uses standard [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/get-started/index.html) tooling, targeting the ESP32-C3.

```bash
# One-time setup: source the ESP-IDF export script
. $IDF_PATH/export.sh

# Set the target (only needed once per build dir)
idf.py set-target esp32c3

# Build
idf.py build

# Flash + monitor (adjust the port as needed)
idf.py -p /dev/ttyUSB0 flash monitor
```

## Roadmap

- [ ] Eliminate ESP-NOW-induced control loop jitter
- [ ] Move to a more capable MCU for sub-0.5 ms loop timing
- [ ] Add onboard state estimation for swarm-capable operation
- [ ] Replace the `MPU6050`-only sensing path with a more robust/redundant IMU setup

## License

No license file is currently included in this repository — treat the code as "all rights reserved" unless/until a license is added.
