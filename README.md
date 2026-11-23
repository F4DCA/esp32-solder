# esp32-solder

A T12 soldering station on an ESP32-C3, built as an Arduino/PlatformIO port of the ATmega
v1.9 sketch. The control flow, menus, tip handling and EEPROM layout come from the original.
The AVR peripherals are replaced.

`main` holds the port. The earlier ESPHome hybrid is on the `esphome` branch, and `master`
is the upstream mirror with the original hardware files.

## Status

The firmware compiles for the ESP32-C3. Nothing has been flashed and the hardware is not
assembled. The pin map, the calibration and the safety behavior are unverified. Check them
before you connect a heater.

The host simulation and the ESPHome build are on `esphome`.

## Build

```bash
pio run
```

`platformio.ini` uses pioarduino 55.03.312 (Arduino core 3.3.12) and U8g2 from the
PlatformIO registry. The PID library from the original project is vendored under
`software/esp32/lib/PID_v1`. `pio run -t upload` has not been tried; no hardware is
connected.

On `esphome`, the build is `esphome config firmware/t12.yaml` and
`esphome compile firmware/t12.yaml`.

## Pin map

| GPIO | Signal | Part |
|---|---|---|
| 0 | I2C SDA | ADS1115 at 0x48 and SSD1306 at 0x3C |
| 1 | I2C SCL | same bus, 400 kHz |
| 3 | Buzzer | active buzzer, steady level |
| 4 | Heater gate | 2N2222A into the IRF4905 gate, high is on |
| 5 | Encoder A | EC11, internal pull-up |
| 6 | Encoder B | EC11, internal pull-up |
| 7 | Encoder button | EC11 switch, internal pull-up |
| 10 | Handle shake switch | ball or vibration switch, internal pull-up |

GPIO8 and GPIO9 are unused. They are strapping pins and carry the DevKit's LED and BOOT
button. The tip is read through the ADS1115 on AIN0, so it needs no GPIO.

## Hardware

| Part | Note |
|---|---|
| ESP32-C3 DevKitM-1 | 4 MB flash, 320 KB RAM, 5 V in |
| SSD1306 0.96" 128x64 I2C | 0x3C, 3.3 V |
| EC11 rotary encoder | switch on GPIO7 |
| Active buzzer | 3.3 V, steady level |
| Ball or vibration switch | handle movement, GPIO10 |
| ADS1115 | 0x48, PGA +/-4.096 V |
| MCP6002 or OPA2330 | 3-point front end, 3.3 V |
| IRF4905 and 2N2222A | P-channel high side, 2.2k/2.2k gate divider, 200k gate-source |
| 24 V 5 A supply and XL4015 buck | 5 V for the DevKit |
| 1000 uF/35 V and 100 nF | 24 V rail |
| 2x 1.5 k or 2.2 k | I2C pull-ups to 3.3 V, in parallel with the module pull-ups |
| T12 handle, GX12-5, BC1.5 | cartridge and cable |

The heater switches the high side. The T12 cartridge shares its two leads between the heater
and the thermocouple, and the amplifier reads the tip lead against ground. A low-side switch
would leave that lead at 24 V while the ADC reads it. GPIO4 high turns the heater on.

## What changed from the original sketch

| Original | Port |
|---|---|
| Internal ADC on A0 | ADS1115 over I2C |
| Timer1 `analogWrite` | LEDC, 8-bit, 1 kHz |
| EEPROM library | NVS behind an EEPROM shim |
| U8glib | U8g2 |
| N-channel charge pump | P-channel high side |
| 4 kHz buzzer square wave | steady level for an active buzzer |
| `sizeof` menu counts | counts from the element size |
| VIN, VCC, chip temperature | stubs and the C3 sensor |

The gate divider is 2.2k/2.2k with a 200k gate-source. The port runs the heater at 1 kHz,
which is why the divider is that low. The ESPHome branch's 100 Hz software PWM would run on
the original 10k/6.8k pair, but the 2.2k/2.2k pair works with both.

`software/esp32/README.md` has the detailed port notes.

## Wiring

`WIRING.md` has the net-by-net table, the component pin map and a bring-up order.
The KiCad schematic, its renders and the netlist are kept local for now and will be
published once the layout is finished. The original v2.7 schematic is in `hardware/`, kept
for the analog front end.

The handle's 5-pin pinout is not verified. The switch is assumed to return to P-. Confirm
P+, P-, SW, E and NTC with a meter before wiring GPIO10.

## Calibration

The default CalTemp table is the upstream one. Measure the ADC counts at 200, 280 and
360 C and update the constants in `software/esp32/src/SolderingStation2_esp32.ino`, or use
the calibration screen, which stores them in NVS. Do not trust the defaults.

The MCP6002's offset at 3.3 V maps the op-amp rail to about 450 C, so the 500 C tip-removal
screen may not appear. Changing R9 to 2 k lowers the gain to 101 and fixes that; recalibrate
afterward. The wiring diagram calls that resistor R8.

## Safety

The firmware drives the heater off before its output is enabled. The 10 k base pull-down
and the gate pull-up hold the FET off while the MCU is in reset. There is no hardware
over-temperature cutoff. A stuck GPIO, NPN or FET holds 2.8 A, and the 5 A fuse does not
open at that current. Add a thermal fuse if the station runs unattended.

The 24 V supply, the buck and the heater stage meet at the bulk capacitor. Keep the heater
return separate from the signal ground until that point.

## License and lineage

The software is ported from the ATmega T12 project and stays under CC-BY-SA 3.0. See
`LICENSE`. The original sketch is in `software/original/sources/`.
