# esp32-solder

A T12 soldering station on a single ESP32-C3: 24 V MOSFET heater stage, thermocouple front
end, 128x64 OLED and a rotary encoder. No cloud service is involved and the iron works
without a network.

> **Work in progress, not tested yet.** Only the control code and the host simulation have
> been run; the firmware compiles but has never been flashed, and the hardware has not been
> assembled. The pinout, the calibration and the safety interlocks are unverified. Check
> them yourself before you connect a heater.

## Status

| Area | State |
|---|---|
| Control code and host simulation | verified, numbers below |
| Firmware | ESPHome config plus the `t12` component, compiles, not flashed |
| Hardware | not assembled |

## Control model

`lib/control` is HAL-free C++ shared by the host simulation and the firmware:

- parallel-form PID with derivative on measurement and conditional-integration anti-windup,
  output clamped to 0..1 duty;
- `heater_timing` owns the 100 Hz frame, the heater-off measurement window, the front-end
  calibration (3-point op-amp table or a direct thermocouple), the operating modes and the
  fault latch;
- first-order model of the sensed tip node in `lib/sim/plant.cpp`: `Rth = 6.11 K/W`,
  `Cth = 0.245 J/K` (tau = 1.5 s), `R_tip = 8.5 ohm`, 24 V gives 67.8 W, and the DC gain is
  414 °C per unit duty;
- gains seeded for zeta = 1 at 1 rad/s: `Kp = 0.0048`, `Ki = 0.0036`, `Kd = 0`.

Measured step response, 26 to 350 °C with 100 Hz control and a heater-off sample every fifth
frame, over 30 s: rise time (10 to 90 %) 2.41 s, settling inside 2 % after 4.15 s, overshoot
0.0 %, final error +0.09 °C. Duty is 0.79053 on control frames, 0.75 on sample frames and
0.78275 averaged over the cycle.

## Timing constraint

A T12 cartridge shares its two leads between heater and thermocouple, so the heater has to be
off while the temperature is read. Settling and conversion take about 2.5 ms per measurement,
which limits how much of each frame is left for heating:

| Sampling | Overhead | Max duty | Max temperature |
|---|---|---|---|
| 100 Hz, every frame | 25 % | 0.75 | 337 °C |
| 50 Hz frames | 12.5 % | 0.875 | 388 °C |
| 100 Hz control, 20 Hz sampling | 5 % | 0.95 | 419 °C |

That is why the PID runs at 100 Hz while the thermocouple is sampled at 20 Hz. The 0.09 °C
of steady-state error comes from the same place: the sample catches the temperature at the
top of the cycle, and the clipped sample frame loses a little energy.

## Build and run

```bash
pio run                        # host simulation (env:native)
./.pio/build/native/program    # 100 Hz control, 30 s step -> step_response.csv
./.pio/build/native/program --self-test   # calibration, duty ceiling, faults, modes
python3 tools/plot_step.py     # metrics, ASCII plot -> step_response.svg
```

No hardware is needed. The CSV and the SVG are generated, so neither is tracked.

The firmware is built by ESPHome, not PlatformIO:

```bash
esphome config  firmware/t12.yaml
esphome compile firmware/t12.yaml
```

`firmware/secrets.yaml` stays out of the repository. Create it next to the YAML with
`ap_password`, `api_key` and `web_password` before the first build.

## Layout

```
lib/control/       HAL-independent PID, frame scheduler, interlock and calibration
lib/sim/           first-order thermal plant, host simulation only
src/sim_main.cpp   host harness: 100 Hz control, 20 Hz sampling, 30 s, 7-column CSV
components/t12/    ESPHome component: ADS1115 read, heater PWM, encoder, OLED
firmware/t12.yaml  ESP32-C3 configuration
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
| MCP6002 or OPA2330 op-amp | amplifier for the default 3-point front end; the direct thermocouple mode skips it |
| IRLZ44N and 2N2222A | logic-level N-MOSFET, low-side heater switch, with a 10 k pull-down on the base so the FET stays off while the MCU is in reset |
| 24 V, 5 A supply with a 5 A slow-blow fuse | buck converter down to 5 V for the MCU |
| T12 handle, 5-pin GX16 cable, T12 tip | cartridge only |

The heater is on GPIO4 through the inverting 2N2222A stage, I2C on GPIO0 and GPIO1, the
encoder on GPIO5 and GPIO6, the encoder button on GPIO7, the handle shake switch on GPIO10
and the buzzer on GPIO3. GPIO8 and GPIO9 are strapping pins on the C3 and also carry the
DevKitM-1 RGB LED and BOOT button, so the bus avoids them.

## Safety

The firmware drives the heater off before its output is enabled and latches it off on a
sensor error, a missing tip or over-temperature. The gate driver has to hold the FET off
while the MCU is in reset, which is what the base pull-down is for. Trip points live in
`lib/control/heater_timing.h`, and a remote setpoint goes through the same 150 to 400 °C
clamp as the encoder.

Wi-Fi is off at boot and switched on from the setup menu. With the radio off, the iron runs
from the encoder and the display; the web UI, the API and OTA are reachable only while Wi-Fi
is on.

## Third-party material

`reference/` holds firmware and schematics by their original authors, kept for reference and
not built. The upstream work is covered by `LICENSE`.
