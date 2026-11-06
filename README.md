# esp32-solder

A T12 soldering station on a single ESP32-C3: 24 V MOSFET heater stage, thermocouple front
end, 128x64 OLED and a rotary encoder. The design has no cloud component and needs no
account.

## Status

| Area | State |
|---|---|
| Control code and host simulation | verified, numbers below |
| Firmware | in progress |
| Hardware | not assembled, nothing flashed yet |

## Control model

`lib/control` is HAL-free C++ shared by the host simulation and the firmware:

- parallel-form PID with derivative on measurement and conditional-integration anti-windup,
  output clamped to 0..1 duty;
- first-order model of the sensed tip node: `Rth = 6.11 K/W`, `Cth = 0.245 J/K` (tau = 1.5 s),
  `R_tip = 8.5 ohm`, 24 V gives 67.8 W, and the DC gain is 414 °C per unit duty;
- gains seeded analytically for zeta = 1 at 1 rad/s: `Kp = 0.0048`, `Ki = 0.0036`,
  `Kd = 0`.

Measured step response, 26 to 350 °C at 100 Hz over 30 s: rise time (10 to 90 %) 2.32 s,
settling within 2 % after 4.15 s, no overshoot, final error 0.00 °C, steady duty 0.783.

## Build and run

```bash
pio run                        # builds the host simulation (env:native)
./.pio/build/native/program    # 100 Hz, 30 s step -> step_response.csv
python3 tools/plot_step.py     # metrics, ASCII plot -> step_response.svg
```

No hardware is needed. The CSV and the SVG are generated, so neither is tracked.

## Layout

```
lib/control/       HAL-independent PID and thermal plant, shared by sim and firmware
src/sim_main.cpp   host harness: 100 Hz, 30 s, writes a 7-column CSV
src/main.cpp       firmware entry point (stub)
tools/plot_step.py dependency-free metrics, ASCII plot and SVG output
reference/         third-party firmware and schematics, kept for reference only
```

## Hardware

| Part | Note |
|---|---|
| ESP32-C3 | single MCU, 4 MB flash, 320 KB RAM |
| SSD1306 0.96" 128x64 I2C OLED | 4-pin module |
| EC11 rotary encoder with switch | or KY-040 module with its pull-ups removed |
| ADS1115, 16-bit I2C ADC | thermocouple front end |
| IRLZ44N and 2N2222A | logic-level N-MOSFET, low-side heater switch |
| 24 V, 5 A supply with a 5 A slow-blow fuse | buck converter down to 5 V for the MCU |
| T12 handle, 5-pin GX16 cable, T12 tip | cartridge only |

## Timing constraint

A T12 cartridge shares its two leads between heater and thermocouple, so the heater has to be
off while the temperature is read. Settling and conversion take about 2.5 ms per measurement
frame, which limits how much of each frame is left for heating:

| Sampling | Overhead | Max duty | Max temperature |
|---|---|---|---|
| 100 Hz, every frame | 25 % | 0.75 | 337 °C |
| 50 Hz frames | 12.5 % | 0.875 | 388 °C |
| 100 Hz control, 20 Hz sampling | 5 % | 0.95 | 419 °C |

The PID therefore runs at 100 Hz while the thermocouple is sampled at 20 Hz.

## Safety

The heater stays off on boot, on fault and on watchdog reset. The firmware also has to detect
a disconnected tip and cut off at a maximum temperature. A remote setpoint uses the same
limits as the local one.

## Third-party material

`reference/` holds firmware and schematics by their original authors, kept for reference and
not built. The upstream work is covered by `LICENSE`.
