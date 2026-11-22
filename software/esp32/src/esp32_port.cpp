#include "esp32_port.h"

#include <cmath>
#include <cstring>

#include "soc/soc_caps.h"

EEPROMClass EEPROM;

// ---------------------------------------------------------------------------
// Rotary encoder
// ---------------------------------------------------------------------------
volatile uint8_t a0, b0, c0, d0;
volatile bool ab0;
volatile int count, countMin, countMax, countStep;
volatile bool handleMoved;

namespace {

uint8_t rotary_pin_a = 255;
uint8_t rotary_pin_b = 255;

void IRAM_ATTR rotaryISR() {
  const uint8_t a = gpio_get_level(static_cast<gpio_num_t>(rotary_pin_a));
  const uint8_t b = gpio_get_level(static_cast<gpio_num_t>(rotary_pin_b));

  if (a != a0) {              // A changed
    a0 = a;
    if (b != b0) {            // B changed
      b0 = b;
      count = constrain(count + ((a == b) ? countStep : -countStep), countMin, countMax);
      if (ROTARY_TYPE && ((a == b) != ab0)) {
        count = constrain(count + ((a == b) ? countStep : -countStep), countMin, countMax);
      }
      ab0 = (a == b);
      handleMoved = true;
    }
  }
}

}  // namespace

void rotaryBegin(uint8_t pin_a, uint8_t pin_b) {
  rotary_pin_a = pin_a;
  rotary_pin_b = pin_b;
  a0 = gpio_get_level(static_cast<gpio_num_t>(pin_a));
  b0 = gpio_get_level(static_cast<gpio_num_t>(pin_b));
  ab0 = (a0 == b0);
  attachInterrupt(digitalPinToInterrupt(pin_a), rotaryISR, CHANGE);
}

// ---------------------------------------------------------------------------
// ADS1115
// ---------------------------------------------------------------------------
namespace {

bool adsWrite16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(ADS1115_ADDRESS);
  Wire.write(reg);
  Wire.write(static_cast<uint8_t>(value >> 8));
  Wire.write(static_cast<uint8_t>(value));
  return Wire.endTransmission() == 0;
}

bool adsRead16(uint8_t reg, uint16_t *value) {
  Wire.beginTransmission(ADS1115_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADS1115_ADDRESS, static_cast<size_t>(2)) != 2) return false;
  *value = (static_cast<uint16_t>(Wire.read()) << 8) | static_cast<uint8_t>(Wire.read());
  return true;
}

}  // namespace

uint16_t adsReadTip(uint8_t channel) {
  // Single-ended MUX: AIN0 = 100b, AIN1 = 101b, AIN2 = 110b, AIN3 = 111b.
  const uint16_t mux = static_cast<uint16_t>((0x04 | (channel & 0x03)) << 12);
  const uint16_t config = static_cast<uint16_t>((ADS1115_CFG_SINGLE & ~0x7000) | mux);
  // A dead bus reads as a removed tip, not as a cold one: 65535 maps far above
  // the 500 C tip-removal trip, so the heater stays off instead of heating
  // against a fiction of 21 C.
  if (!adsWrite16(ADS1115_REG_CONFIG, config)) return 65535;

  // Poll the OS bit instead of trusting a fixed conversion delay: at 860 SPS a
  // conversion takes 1.16 ms, and the caller has already allowed 950 us for the
  // front end to settle.
  uint16_t status = 0;
  const uint32_t start = micros();
  do {
    if (!adsRead16(ADS1115_REG_CONFIG, &status)) return 65535;
    if (status & 0x8000) break;
  } while (micros() - start < 4000);
  if (!(status & 0x8000)) return 65535;  // conversion never finished

  uint16_t raw = 0;
  if (!adsRead16(ADS1115_REG_CONVERSION, &raw)) return 65535;

  // Sign-extend the 16-bit conversion, then express it in upstream 10-bit
  // counts so the CalTemp table applies unchanged.
  const int16_t signed_raw = static_cast<int16_t>(raw);
  const double counts10 = signed_raw * ADC16_TO_ADC10;
  if (counts10 <= 0.0) return 0;
  if (counts10 > 65535.0) return 65535;
  return static_cast<uint16_t>(counts10 + 0.5);
}

double portChipTemperatureC() {
#if defined(SOC_TEMP_SENSOR_SUPPORTED) && SOC_TEMP_SENSOR_SUPPORTED
  const float t = temperatureRead();
  if (isfinite(t)) return t;
#endif
  return 25.0;
}

// ---------------------------------------------------------------------------
// EEPROM replacement
// ---------------------------------------------------------------------------
void EEPROMClass::begin(size_t size) {
  if (size > sizeof(data_)) size = sizeof(data_);
  size_ = size;
  prefs_.begin("t12", false);

  const size_t stored = prefs_.getBytesLength("eeprom");
  if (stored == size_) {
    prefs_.getBytes("eeprom", data_, size_);
  } else {
    // Fresh NVS: match an erased AVR EEPROM so getEEPROM() writes its defaults.
    std::memset(data_, 0xFF, size_);
    dirty_ = true;
    commit();
  }
}

void EEPROMClass::write(int idx, uint8_t value) {
  if (idx < 0 || static_cast<size_t>(idx) >= size_) return;
  data_[idx] = value;
  dirty_ = true;
}

void EEPROMClass::update(int idx, uint8_t value) {
  if (idx < 0 || static_cast<size_t>(idx) >= size_) return;
  if (data_[idx] == value) return;
  data_[idx] = value;
  dirty_ = true;
}

void EEPROMClass::commit() {
  if (!dirty_ || size_ == 0) return;
  prefs_.putBytes("eeprom", data_, size_);
  dirty_ = false;
}
