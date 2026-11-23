# Wiring diagram, ESP32-C3 T12 station

One connection per row. The build uses a 2.2k/2.2k gate divider with a 200k gate-source, a 1000 uF bulk cap, and 400 kHz I2C with 1.5k/2.2k pull-ups (100 kHz without them on the port). These supersede the earlier 10k/6.8k and 470 uF values.

## Build checklist

- [ ] Gate divider 2.2k/2.2k, 0.25 W, plus 200k gate-source.
- [ ] Bulk cap 1000uF/35V or 2x470uF/35V.
- [ ] I2C pull-ups 2x1.5k or 2.2k to 3.3V; keep 400kHz on both firmwares.
- [ ] MCP6002 and 3-point calibration; not LMV358.
- [ ] Optional gain 101: R9 2k on the upstream schematic, R8 2k in this diagram; recalibrate.
- [ ] Verify handle pinout and tip-shell isolation.
- [ ] No thermal cutoff fitted; add one for unattended use.

## Nets

| Net | Connections | Notes |
|---|---|---|
| `+24V` | F1.2, C4.+, C5.1, F2.1, Q1.S, D2.K, R1.1, R5.2 | after F1; heater loop and buck input |
| `+3V3` | C6.+, U1.3V3, DS1.VCC, U2.VDD, U3.8 V+, R9.1, C7.1, C8.1, C9.1 | DevKit 3V3 output only; do not back-feed |
| `+5V` | PS2.OUT+, U1.5V | buck output to DevKit 5V; disconnect USB while the buck is on |
| `BUCK_IN` | F2.2, PS2.IN+ | after the 0.5 A buck fuse |
| `BUZZ` | U1.GPIO3, BZ1.+ | active buzzer, steady high |
| `EARTH` | J1.E | handle shell / chassis; bond to GND only if the 24 V supply is earth-referenced |
| `ENC_A` | U1.GPIO5, ENC1.A | encoder A, internal pull-up |
| `ENC_B` | U1.GPIO6, ENC1.B | encoder B, internal pull-up |
| `ENC_SW` | U1.GPIO7, ENC1.SW1 | encoder switch to ground, internal pull-up |
| `GATE` | Q1.G, R1.2, R2.1, R5.1 | FET gate node; R1 pull-up, R2 collector, R5 gate-source |
| `GND` | PS1.-, C4.-, C5.2, PS2.IN-, PS2.OUT-, C6.-, U1.GND, DS1.GND, ENC1.C, ENC1.SW2, BZ1.-, U2.GND, U2.ADDR, U3.4 GND, R8.2, R10.2, C1.2, C2.2, C3.2, D1.A, C7.2, C8.2, C9.2, R4.2, Q2.E, J1.P- | star at C4 negative; heater return joins here, not at the DevKit |
| `GPIO4` | U1.GPIO4, R3.1 | heater base drive; must be low at reset |
| `INV1` | U3.2 -IN1, R7.1, R8.1 | op-amp -IN1; R7 feedback, R8 to ground |
| `NC` | U2.ALRT, U2.AIN1, U2.AIN2, U2.AIN3, J1.NTC | not connected: U2.ALRT, U2.AIN1-3, J1.NTC |
| `NPN_B` | R3.2, R4.1, Q2.B | base node; R3 from GPIO4, R4 to ground |
| `NPN_C` | R2.2, Q2.C | R2 bottom to Q2 collector |
| `P_LEAD` | R6.1, C1.1, Q1.D, D2.A, J1.P+ | switched tip node: FET drain, T12 P+, C1, R6, D2 anode |
| `SCL` | U1.GPIO1, DS1.SCL, U2.SCL | I2C clock, 3.3 V, 400 kHz with 1.5k/2.2k pull-ups |
| `SDA` | U1.GPIO0, DS1.SDA, U2.SDA | I2C data, 3.3 V, 400 kHz with 1.5k/2.2k pull-ups |
| `SHAKE` | U1.GPIO10, J1.SW | handle switch to ground, internal pull-up |
| `TC_OUT` | U2.AIN0, U3.1 OUT1, R7.2 | op-amp OUT1 to ADS AIN0 |
| `TIP_AMP` | U3.3 +IN1, R6.2, C2.1, D1.K | op-amp +IN1; protected by D1 and filtered by C2 |
| `VIN_24` | PS1.+, F1.1 | supply positive before the fuse |
| `VREF` | U3.5 +IN2, R9.2, R10.1, C3.1 | U3B +IN2 mid-rail, unused buffer |
| `VREF_BUF` | U3.6 -IN2, U3.7 OUT2 | U3B output tied to -IN2; not used elsewhere |

## Component pin map

| Ref | Part | Pin | Net |
|---|---|---|---|
| BZ1 | active buzzer | + | `BUZZ` |
| BZ1 | active buzzer | - | `GND` |
| C1 | 1uF/35V | 1 | `P_LEAD` |
| C1 | 1uF/35V | 2 | `GND` |
| C2 | 2.2nF | 1 | `TIP_AMP` |
| C2 | 2.2nF | 2 | `GND` |
| C3 | 100nF | 1 | `VREF` |
| C3 | 100nF | 2 | `GND` |
| C4 | 1000uF/35V | + | `+24V` |
| C4 | 1000uF/35V | - | `GND` |
| C5 | 100nF/50V | 1 | `+24V` |
| C5 | 100nF/50V | 2 | `GND` |
| C6 | 10uF/10V | + | `+3V3` |
| C6 | 10uF/10V | - | `GND` |
| C7 | 100nF | 1 | `+3V3` |
| C7 | 100nF | 2 | `GND` |
| C8 | 100nF | 1 | `+3V3` |
| C8 | 100nF | 2 | `GND` |
| C9 | 100nF | 1 | `+3V3` |
| C9 | 100nF | 2 | `GND` |
| D1 | 1N4148 | A | `GND` |
| D1 | 1N4148 | K | `TIP_AMP` |
| D2 | SS54 optional | A | `P_LEAD` |
| D2 | SS54 optional | K | `+24V` |
| DS1 | SSD1306 128x64 | VCC | `+3V3` |
| DS1 | SSD1306 128x64 | GND | `GND` |
| DS1 | SSD1306 128x64 | SDA | `SDA` |
| DS1 | SSD1306 128x64 | SCL | `SCL` |
| ENC1 | EC11 encoder | A | `ENC_A` |
| ENC1 | EC11 encoder | B | `ENC_B` |
| ENC1 | EC11 encoder | C | `GND` |
| ENC1 | EC11 encoder | SW1 | `ENC_SW` |
| ENC1 | EC11 encoder | SW2 | `GND` |
| F1 | 5A fuse | 1 | `VIN_24` |
| F1 | 5A fuse | 2 | `+24V` |
| F2 | 0.5A fuse | 1 | `+24V` |
| F2 | 0.5A fuse | 2 | `BUCK_IN` |
| J1 | T12 handle 5-pin | P+ | `P_LEAD` |
| J1 | T12 handle 5-pin | P- | `GND` |
| J1 | T12 handle 5-pin | SW | `SHAKE` |
| J1 | T12 handle 5-pin | E | `EARTH` |
| J1 | T12 handle 5-pin | NTC | `NC` |
| PS1 | 24V 5A supply | + | `VIN_24` |
| PS1 | 24V 5A supply | - | `GND` |
| PS2 | XL4015 buck | IN+ | `BUCK_IN` |
| PS2 | XL4015 buck | IN- | `GND` |
| PS2 | XL4015 buck | OUT+ | `+5V` |
| PS2 | XL4015 buck | OUT- | `GND` |
| Q1 | IRF4905 | G | `GATE` |
| Q1 | IRF4905 | S | `+24V` |
| Q1 | IRF4905 | D | `P_LEAD` |
| Q2 | 2N2222A | B | `NPN_B` |
| Q2 | 2N2222A | C | `NPN_C` |
| Q2 | 2N2222A | E | `GND` |
| R1 | 2.2k | 1 | `+24V` |
| R1 | 2.2k | 2 | `GATE` |
| R10 | 10k | 1 | `VREF` |
| R10 | 10k | 2 | `GND` |
| R2 | 2.2k | 1 | `GATE` |
| R2 | 2.2k | 2 | `NPN_C` |
| R3 | 4.7k | 1 | `GPIO4` |
| R3 | 4.7k | 2 | `NPN_B` |
| R4 | 10k | 1 | `NPN_B` |
| R4 | 10k | 2 | `GND` |
| R5 | 200k | 1 | `GATE` |
| R5 | 200k | 2 | `+24V` |
| R6 | 10k | 1 | `P_LEAD` |
| R6 | 10k | 2 | `TIP_AMP` |
| R7 | 200k | 1 | `INV1` |
| R7 | 200k | 2 | `TC_OUT` |
| R8 | 1k | 1 | `INV1` |
| R8 | 1k | 2 | `GND` |
| R9 | 10k | 1 | `+3V3` |
| R9 | 10k | 2 | `VREF` |
| U1 | ESP32-C3 DevKitM-1 | 5V | `+5V` |
| U1 | ESP32-C3 DevKitM-1 | 3V3 | `+3V3` |
| U1 | ESP32-C3 DevKitM-1 | GND | `GND` |
| U1 | ESP32-C3 DevKitM-1 | GPIO0 | `SDA` |
| U1 | ESP32-C3 DevKitM-1 | GPIO1 | `SCL` |
| U1 | ESP32-C3 DevKitM-1 | GPIO3 | `BUZZ` |
| U1 | ESP32-C3 DevKitM-1 | GPIO4 | `GPIO4` |
| U1 | ESP32-C3 DevKitM-1 | GPIO5 | `ENC_A` |
| U1 | ESP32-C3 DevKitM-1 | GPIO6 | `ENC_B` |
| U1 | ESP32-C3 DevKitM-1 | GPIO7 | `ENC_SW` |
| U1 | ESP32-C3 DevKitM-1 | GPIO10 | `SHAKE` |
| U2 | ADS1115 0x48 | VDD | `+3V3` |
| U2 | ADS1115 0x48 | GND | `GND` |
| U2 | ADS1115 0x48 | SCL | `SCL` |
| U2 | ADS1115 0x48 | SDA | `SDA` |
| U2 | ADS1115 0x48 | ADDR | `GND` |
| U2 | ADS1115 0x48 | ALRT | `NC` |
| U2 | ADS1115 0x48 | AIN0 | `TC_OUT` |
| U2 | ADS1115 0x48 | AIN1 | `NC` |
| U2 | ADS1115 0x48 | AIN2 | `NC` |
| U2 | ADS1115 0x48 | AIN3 | `NC` |
| U3 | MCP6002 | 1 OUT1 | `TC_OUT` |
| U3 | MCP6002 | 2 -IN1 | `INV1` |
| U3 | MCP6002 | 3 +IN1 | `TIP_AMP` |
| U3 | MCP6002 | 4 GND | `GND` |
| U3 | MCP6002 | 5 +IN2 | `VREF` |
| U3 | MCP6002 | 6 -IN2 | `VREF_BUF` |
| U3 | MCP6002 | 7 OUT2 | `VREF_BUF` |
| U3 | MCP6002 | 8 V+ | `+3V3` |

## Power path and star ground

```
PS1 + --> F1(5A) --> +24V --+--> Q1 source
                           +--> F2(0.5A) --> PS2 buck --> +5V --> U1 5V
                           +--> C4 1000uF/35V + C5 100nF
PS1 - ---------------------+--> GND star at C4 negative
T12 P- (2.8 A return) -----+--> GND star
U1 3V3 --> +3V3 --> DS1, U2, U3, R9, C6, C7-C9
```

The heater return carries 2.8 A. Run it as its own 20 AWG pair to the C4 negative terminal, and take the signal ground from the same point. Do not daisy-chain the heater return through the DevKit ground.

## Bring-up order

1. F1 in holder, 24 V off. Check no short from +24 V to GND.
2. Buck output 5.0 V with no DevKit connected. Then connect the DevKit with USB unplugged.
3. Check +3V3 at the OLED/ADS/op-amp. Current should be about 70 mA with the buzzer off.
4. GPIO4 low: Q1 gate at +24 V, T12 P+ at 0 V. GPIO4 high: gate about +12 V, Vgs about -12 V.
5. With the tip fitted and cold, the ADS should read a sane low temperature. Calibrate at 200/280/360 C before soldering.
6. First heat-up with a series meter or a current-limited supply. Expect 2.8 A at full duty.

## KiCad

The KiCad schematic, its SVG/PDF exports and the netlist are kept local for now. They were
validated with `kicad-cli 10.0.8`: `sch export netlist` succeeds and the result matches the
table above pin for pin. They will be published once the layout is finished.

Refs R6/R7/R8 in this diagram are 10k/200k/1k. The upstream schematic calls the same three
R7/R8/R9. To use gain 101 with the MCP6002, change R8 from 1k to 2k and recalibrate.

## Open items

- The handle's 5-pin aviator pinout is unverified. Confirm P+, P-, SW, E, NTC with a meter. The switch may return to P- or to E; the firmware expects it to pull GPIO10 to GND.
- The T12 heater must be isolated from the tip shell. Check with a meter before bonding the shell to earth.
- The earlier 10k/6.8k gate divider and 470 uF bulk cap are superseded by this diagram.
- 400 kHz is the normal fast-mode rate and both chips support it. The modules' 4.7k pull-ups are
  the only issue: add 1.5k/2.2k to 3.3 V and both firmwares can stay at 400 kHz. Without them,
  100 kHz is safe for the port. The ESPHome branch parks the heater during its 500 ms display
  flush, so 100 kHz costs about 18 percent duty there; keep 400 kHz or raise its display
  interval to 1 s.
