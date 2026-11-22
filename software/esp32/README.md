# ESP32-C3 port

The upstream ATmega sketch in `software/original/sources/` rebuilt for the ESP32-C3 with
the smallest set of hardware changes that makes it run on this board. The control flow,
the menus, the screens and the EEPROM layout are upstream v1.9. No features were added.

Software is ported from the ATmega project and stays under CC-BY-SA 3.0, see `LICENSE`.

## Build

```bash
pio run
```

The environment uses pioarduino 55.03.312 (Arduino core 3.3.12, ESP-IDF 5.5) and U8g2 from
the PlatformIO registry. The PID library from upstream's `software/libraries` is vendored
under `lib/PID_v1`, so the build does not fetch it.

Nothing was flashed or run on hardware. A clean compile is the only verified step.

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

GPIO8 and GPIO9 stay unused because they are strapping pins and carry the DevKit's LED and
BOOT button. The tip is read through the ADS1115 on AIN0, so no GPIO is needed for it.

## What changed from upstream

| Upstream | Here | Why |
|---|---|---|
| Internal ADC on A0 | ADS1115 over I2C | The C3 has no free ADC pin with this map, and the parts list already carries the ADS1115 and the MCP6002 front end. One 860 SPS single-shot read is scaled to the 10-bit counts the CalTemp table expects. |
| Timer1 `analogWrite` | LEDC, 8-bit, 1 kHz | The 0..255 duty and the HEATER_ON/OFF/PWM macros stay as they were. The off settle is 2 ms instead of 950 us because a duty change reaches the pin at the next 1 kHz period boundary. |
| EEPROM library | NVS blob behind an EEPROM shim | Same byte offsets and the same `update()` calls. One commit per `updateEEPROM()` instead of one write per byte. |
| U8glib | U8g2 | Same drawing calls. `setPrintPos` became `setCursor`, the font names changed, and screen flip uses `setDisplayRotation`. |
| N-channel MOSFET | P-channel | Matches the IRF4905 high-side stage on this board, so GPIO4 high turns the heater on. |
| 4 kHz square wave on the buzzer | steady high for 64 ms | The board fits an active buzzer. 64 ms is the length of the original burst. |
| AVR chip temperature, Vcc, Vin | C3 sensor, 3.3 V stub, 24.0 V stub | There is no input divider on this board. The Info screen reports the nominal values, not measurements. |
| `sizeof(Items)` halved in `MenuScreen` | `ITEM_COUNT` | AVR pointers are two bytes and ESP32 pointers are four, so the original halving would have doubled every menu count. |

The degree glyph in the temperature item labels was dropped because the U8g2 9x15 font has no
0xB0, so those screens read `150 C` instead of `150 °C`.

A failed ADS1115 transfer reads as 65535 counts, well above the 500 C tip-removal trip, so a
dead bus leaves the heater off instead of heating against a false cold reading. There is no
latched fault state; upstream does not have one either.

## Not verified

All of this needs hardware:

- the pin map on a real DevKitM-1
- ADS1115 scaling at 200, 280 and 360 C
- encoder direction and detents per step
- OLED orientation and the screen flip setting
- the active buzzer's loudness and pitch
- EEPROM settings surviving a power cycle

No flashing was done.
