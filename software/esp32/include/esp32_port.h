#pragma once
// ESP32-C3 hardware adaptation for the upstream ATmega soldering station.
//
// The sketch keeps the upstream control flow and menus. Only the AVR hardware
// layer is replaced here:
//
//   ADC      -> ADS1115 over I2C (single-shot, 860 SPS, PGA +-4.096 V)
//   EEPROM   -> Preferences/NVS, same byte offsets and the same update() call
//   chip temp-> the C3 internal temperature sensor
//   VIN/VCC  -> stubs, because the board has no input-voltage divider
//
// The ADS1115 returns 16-bit counts on a 5 V scale. Scaling by 0.125 V per
// count and then by 1023/5 V keeps the upstream CalTemp table (216/308/390)
// valid without touching calculateTemp().

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <driver/gpio.h>

// ADS1115: AIN0 against GND, PGA +-4.096 V, single-shot, 860 SPS, comparator
// off. Same word the ESPHome branch uses for its op-amp front end.
static constexpr uint8_t ADS1115_ADDRESS = 0x48;
static constexpr uint8_t ADS1115_REG_CONVERSION = 0x00;
static constexpr uint8_t ADS1115_REG_CONFIG = 0x01;
static constexpr uint16_t ADS1115_CFG_SINGLE = 0xC3E3;

// One ADS1115 count is 125 uV; one upstream (10-bit, 5 V) count is 4.888 mV.
static constexpr double ADC16_TO_ADC10 = 0.125 / (5000.0 / 1023.0);

// No divider is fitted, so the Info screen reports the nominal supply and the
// 3.3 V rail instead of inventing a measurement. See software/esp32/README.md.
static constexpr uint16_t VIN_STUB_MV = 24000;
static constexpr uint16_t VCC_STUB_MV = 3300;

// Rotary encoder: 0 = 2 increments/step, 1 = 4 increments/step (default).
#define ROTARY_TYPE 1

// Quadrature state, written from the encoder ISR and read by the sketch.
extern volatile uint8_t a0, b0, c0, d0;
extern volatile bool ab0;
extern volatile int count, countMin, countMax, countStep;
extern volatile bool handleMoved;

// Sets the quadrature pins, seeds a0/b0/ab0 and attaches the ISR. The handler
// lives in esp32_port.cpp so IRAM_ATTR is not lost to the .ino prototype pass.
void rotaryBegin(uint8_t pin_a, uint8_t pin_b);

// Reads one ADS1115 single-ended channel (0 = tip, 1 = unused) and returns the
// upstream 10-bit equivalent. Returns 65535 if the I2C transfer failed or the
// conversion timed out, which lands above the 500 C tip-removal trip so the
// heater stays off instead of heating against a false cold reading.
uint16_t adsReadTip(uint8_t channel);

// C3 internal temperature, in C. The sensor is uncalibrated; it is display only.
double portChipTemperatureC();

// EEPROM replacement backed by one NVS blob. Same read/update interface as the
// AVR library so getEEPROM() and updateEEPROM() stay byte-for-byte upstream.
class EEPROMClass {
 public:
  void begin(size_t size = 512);
  uint8_t read(int idx) const { return data_[idx]; }
  void write(int idx, uint8_t value);
  void update(int idx, uint8_t value);
  void commit();

 private:
  Preferences prefs_;
  size_t size_ = 0;
  bool dirty_ = false;
  uint8_t data_[512] = {0};
};

extern EEPROMClass EEPROM;
