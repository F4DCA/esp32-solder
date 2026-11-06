#include "t12_component.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif

namespace esphome {
namespace t12 {

static const char *const TAG = "t12";

// ADS1115 registers and configuration words.
//   OS=1 (start) | MUX | PGA | MODE=1 (single-shot) | DR=111 (860 SPS) | comparator off
//   OPAMP_3POINT: AIN0 vs GND, PGA +-4.096 V, 125 uV/LSB       -> 0xC3E3
//   DIRECT_TC:    AIN0 vs AIN1, PGA +-0.256 V, 7.8125 uV/LSB   -> 0x8FE3
static constexpr uint8_t ADS1115_REG_CONVERSION = 0x00;
static constexpr uint8_t ADS1115_REG_CONFIG = 0x01;
static constexpr uint16_t ADS1115_CFG_OPAMP_3POINT = 0xC3E3;
static constexpr uint16_t ADS1115_CFG_DIRECT_TC = 0x8FE3;

static constexpr double TEMP_STEP_C = 10.0;
static constexpr uint32_t BUTTON_LONG_PRESS_US = 500000;
static constexpr uint8_t MENU_ITEM_COUNT = 4;
static constexpr uint8_t MENU_ROWS = 3;

// 10-bit ADC equivalent of one ADS1115 LSB at PGA +-4.096 V, so the upstream
// CalTemp table (216/308/390) applies unchanged.
static constexpr double ADC16_TO_ADC10 = 0.125 / (5000.0 / 1023.0);

void T12Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up T12 station...");

  const TimingConfig &timing = this->scheduler_.config();
  this->frame_us_ = static_cast<uint32_t>(timing.frame_dt_s * 1e6 + 0.5);
  // Settle and conversion budgets come from the scheduler so the off-window the
  // duty ceiling was derived from is the off-window the hardware actually takes.
  this->settle_us_ = static_cast<uint32_t>(timing.settle_us + 0.5);
  this->conv_us_ = static_cast<uint32_t>(timing.conv_us + 0.5);

  // Heater OFF before anything else. The output is inverted in YAML, so a
  // logical false drives the ESP32 pin HIGH, turns the 2N2222A on and pulls the
  // IRLZ44N gate to GND. The pad is written low before it becomes an output so
  // it is already at the safe level the moment the driver takes over.
  if (this->heater_pin_ != nullptr) {
    this->heater_pin_->digital_write(false);
    this->heater_pin_->setup();
    this->heater_pin_->digital_write(false);
    this->heater_on_ = false;
  }
  if (this->buzzer_pin_ != nullptr) {
    this->buzzer_pin_->setup();
    this->buzzer_pin_->digital_write(false);
  }
  if (this->shake_pin_ != nullptr) {
    this->shake_pin_->setup();
    this->last_shake_ = this->shake_pin_->digital_read();
  }

  this->scheduler_.reset(26.0);
  this->scheduler_.setUserSetpoint(this->default_setpoint_c_);
  this->pid_.setGains(this->kp_, this->ki_, this->kd_);

  this->last_frame_ = FrameResult{};
  this->last_frame_.mode = this->scheduler_.mode();
  this->last_frame_.status = HeaterStatusLabel::Off;
  this->last_frame_.active_setpoint_c = this->scheduler_.activeSetpoint();
  // Until the first measurement lands the station reports its power-on
  // temperature instead of 0 C.
  this->last_frame_.measured_temp_c = 26.0;
  this->last_frame_.display_temp_c = 26.0;

  this->last_frame_us_ = micros();
  this->frame_start_us_ = this->last_frame_us_;
  this->frame_prev_start_us_ = this->last_frame_us_;
  this->applyHeaterDuty_(0.0, this->last_frame_us_);
}

void T12Component::dump_config() {
  ESP_LOGCONFIG(TAG, "T12 station:");
  LOG_PIN("  Heater pin: ", this->heater_pin_);
  LOG_PIN("  Handle shake pin: ", this->shake_pin_);
  LOG_PIN("  Buzzer pin: ", this->buzzer_pin_);
  ESP_LOGCONFIG(TAG, "  Front end: %s",
                this->frontend_cfg_.mode == FrontEndMode::DirectTcLinear ? "direct TC (linear)"
                                                                        : "op-amp 3-point");
  if (this->frontend_cfg_.mode == FrontEndMode::DirectTcLinear) {
    ESP_LOGCONFIG(TAG, "  Seebeck %.1f uV/C, cold junction %.1f C", this->frontend_cfg_.tc_uv_per_c,
                  this->frontend_cfg_.cjc_temp_c);
  } else {
    ESP_LOGCONFIG(TAG, "  Calibration ADC 200/280/360 C: %.0f/%.0f/%.0f",
                  this->frontend_cfg_.cal_200, this->frontend_cfg_.cal_280,
                  this->frontend_cfg_.cal_360);
  }
  ESP_LOGCONFIG(TAG, "  Gains kp=%.5f ki=%.5f kd=%.5f", this->kp_, this->ki_, this->kd_);
  ESP_LOGCONFIG(TAG, "  Frame %.0f Hz, sample every %d frame(s), duty ceiling %.2f",
                1.0 / this->scheduler_.config().frame_dt_s, this->scheduler_.config().sample_stride,
                this->scheduler_.cycleAverageMaxDuty());
  ESP_LOGCONFIG(TAG, "  Default setpoint %.0f C, tip %s, CSV telemetry %s",
                this->default_setpoint_c_, this->tip_name_.c_str(),
                this->csv_telemetry_ ? "on" : "off");
}

void T12Component::loop() {
  const uint32_t now = micros();

  // A stalled pass (blocking I2C or display flush) must never leave the FET
  // asserted: no on-window outlives the frame that opened it. applyHeaterDuty_
  // refreshes the window at every frame boundary, so a pin that stays high
  // across boundaries at full duty is legitimate here and is left alone.
  // The pass that ends a stall leaves the FET off; the next pass re-evaluates
  // against whatever window the frame service opened in between.
  bool stall_cut = false;
  if (this->heater_on_ && (now - this->window_start_us_) > this->window_len_us_ + 2000u) {
    this->forceHeaterOff_();
    stall_cut = true;
  }

  // Network setpoint lands at a frame boundary. It is a plain value change: no
  // mode transition and no idle-timer reset.
  if (this->pending_setpoint_valid_) {
    this->pending_setpoint_valid_ = false;
    this->scheduler_.setUserSetpoint(this->pending_setpoint_c_);
  }

  // Reached again only after the caller of the display lambda, flush included.
  this->display_busy_ = false;

  this->serviceBuzzer_(now);

  if (this->shake_pin_ != nullptr) {
    const bool level = this->shake_pin_->digital_read();
    if (level != this->last_shake_) {
      this->last_shake_ = level;
      this->scheduler_.notifyHandleMoved();
    }
  }

  if (now - this->last_frame_us_ >= this->frame_us_) {
    this->last_frame_us_ += this->frame_us_;
    // Fell more than a frame behind (blocking I2C/display): resync instead of
    // replaying the missed frames back to back.
    if (now - this->last_frame_us_ >= this->frame_us_) this->last_frame_us_ = now;
    this->frame_prev_start_us_ = this->frame_start_us_;
    this->frame_start_us_ = this->last_frame_us_;
    this->serviceFrame_(now);
  }

  if (this->sample_phase_ != SamplePhase::Idle) this->serviceSample_(now);

  // Holding the FET off for the rest of the stalled pass is the point of the
  // cut: re-asserting here would hand a fresh window straight back to the FET.
  if (!stall_cut) this->updateHeaterOutput_();
}

void T12Component::serviceFrame_(uint32_t now_us) {
  // Real interval since the previous frame start, so the station timers and the
  // derivative term follow the actual cadence, resync jumps included.
  const double elapsed_s = static_cast<double>(now_us - this->frame_prev_start_us_) * 1e-6;

  // A stalled loop (the display flush blocks for tens of milliseconds) can leave
  // a measurement window open across a frame boundary. Close it first so the
  // frame that owns it is stepped exactly once, and drop this frame if the
  // conversion is still running.
  if (this->sample_phase_ != SamplePhase::Idle) {
    this->serviceSample_(now_us);
    if (this->sample_phase_ != SamplePhase::Idle) {
      this->in_blanking_window_ = true;
      this->applyHeaterDuty_(0.0, now_us);
      return;
    }
    // finishSample_ stepped the frame that owns this measurement. Deliver its
    // duty over what is left of the missed frame instead of stepping a second
    // frame inside one tick.
    this->applyHeaterDuty_(this->last_frame_.effective_duty, now_us);
    return;
  }

  this->in_blanking_window_ = false;

  if (this->in_setup_menu_) {
    // Menu: heater parked, timers and display keep running.
    this->last_frame_ = this->scheduler_.stepFrameHeld(this->pid_, elapsed_s);
    this->applyHeaterDuty_(0.0, now_us);
    return;
  }

  if (this->scheduler_.isSampleFrame()) {
    // Blanking window opens here; the sample is finished in serviceSample_.
    this->in_blanking_window_ = true;
    this->sample_phase_ = SamplePhase::WaitSettle;
    this->blank_start_us_ = now_us;
    this->applyHeaterDuty_(0.0, now_us);
    return;
  }

  this->last_frame_ = this->scheduler_.stepFrame(this->pid_, this->scheduler_.lastMeasuredTemp(),
                                                 true, elapsed_s);
  this->applyHeaterDuty_(this->last_frame_.post_blank_pwm_duty, now_us);
  if (this->last_frame_.beep_event) this->triggerBeep_(80);
}

void T12Component::serviceSample_(uint32_t now_us) {
  switch (this->sample_phase_) {
    case SamplePhase::WaitSettle:
      if (now_us - this->blank_start_us_ < this->settle_us_) return;
      if (!this->startAdsConversion_()) {
        ESP_LOGW(TAG, "ADS1115 config write failed, latching a sensor fault");
        this->finishSample_(false, this->scheduler_.lastMeasuredTemp());
        return;
      }
      this->conv_start_us_ = micros();
      this->sample_phase_ = SamplePhase::WaitConversion;
      return;

    case SamplePhase::WaitConversion: {
      if (now_us - this->conv_start_us_ < this->conv_us_) return;
      double temp_c = this->scheduler_.lastMeasuredTemp();
      const bool ok = this->readAds1115_(&temp_c);
      if (!ok) ESP_LOGW(TAG, "ADS1115 read failed, latching a sensor fault");
      this->finishSample_(ok, temp_c);
      return;
    }

    case SamplePhase::Idle:
    default:
      return;
  }
}

void T12Component::finishSample_(bool sample_valid, double temp_c) {
  const uint32_t now = micros();
  const double elapsed_s = static_cast<double>(now - this->frame_prev_start_us_) * 1e-6;

  this->last_frame_ = this->scheduler_.stepFrame(this->pid_, temp_c, sample_valid, elapsed_s);
  this->in_blanking_window_ = false;
  this->sample_phase_ = SamplePhase::Idle;
  this->sample_count_++;

  // The heater was off for the blanking window; the remaining time of the frame
  // gets the duty that makes the frame deliver its clipped average.
  this->applyHeaterDuty_(this->last_frame_.post_blank_pwm_duty, now);
  if (this->last_frame_.beep_event) this->triggerBeep_(80);

  // 5 Hz log: one line per 4 completed measurements keeps the serial console
  // quiet enough not to perturb the 100 Hz loop.
  if (this->csv_telemetry_ && (this->sample_count_ % 4u) == 0u) {
    std::printf("%.0f,%.2f,%.3f,%.5f,%.4f,%.4f,%.4f\n", static_cast<double>(now / 1000u),
                this->last_frame_.active_setpoint_c, this->last_frame_.measured_temp_c,
                this->last_frame_.effective_duty, this->pid_.pTerm(), this->pid_.iTerm(),
                this->pid_.dTerm());
  }
}

bool T12Component::startAdsConversion_() {
  const uint16_t config_word = (this->frontend_cfg_.mode == FrontEndMode::DirectTcLinear)
                                   ? ADS1115_CFG_DIRECT_TC
                                   : ADS1115_CFG_OPAMP_3POINT;
  return this->write_byte_16(ADS1115_REG_CONFIG, config_word);
}

bool T12Component::readAds1115_(double *temp_c) {
  uint8_t buffer[2];
  if (!this->read_bytes(ADS1115_REG_CONVERSION, buffer, 2)) return false;

  const int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(buffer[0]) << 8) | buffer[1]);

  double reading;
  if (this->frontend_cfg_.mode == FrontEndMode::DirectTcLinear) {
    reading = raw * 7.8125;  // differential microvolts
  } else {
    reading = raw * ADC16_TO_ADC10;  // 10-bit ADC counts, upstream CalTemp units
  }

  const double sample = rawToTempC(reading, this->frontend_cfg_);
  if (!std::isfinite(sample)) return false;

  if (!this->ema_valid_) {
    this->ema_temp_c_ = sample;
    this->ema_valid_ = true;
  } else {
    const double alpha = std::clamp(this->frontend_cfg_.ema_alpha, 0.01, 1.0);
    this->ema_temp_c_ += (sample - this->ema_temp_c_) * alpha;
  }

  *temp_c = this->ema_temp_c_;
  return true;
}

void T12Component::applyHeaterDuty_(double duty, uint32_t now_us) {
  if (!std::isfinite(duty)) duty = 0.0;
  const uint32_t elapsed = now_us - this->frame_start_us_;
  this->window_len_us_ = (elapsed < this->frame_us_) ? (this->frame_us_ - elapsed) : 0;
  this->window_start_us_ = now_us;
  this->current_pwm_duty_ = std::clamp(duty, 0.0, 1.0);
  // Zero duty means the FET goes off now, not at the next pass: the caller could
  // block for tens of milliseconds between here and there.
  if (this->current_pwm_duty_ <= 0.0) this->forceHeaterOff_();
}

void T12Component::forceHeaterOff_() {
  if (this->heater_pin_ != nullptr && this->heater_on_)
    this->heater_pin_->digital_write(false);
  this->heater_on_ = false;
}

void T12Component::updateHeaterOutput_() {
  if (this->heater_pin_ == nullptr) return;

  bool want = false;
  if (!this->in_blanking_window_ && !this->in_setup_menu_ && !this->display_busy_) {
    const OperatingMode mode = this->scheduler_.mode();
    if (mode != OperatingMode::Off && mode != OperatingMode::Fault) {
      double duty = this->current_pwm_duty_;
      if (!std::isfinite(duty)) duty = 0.0;
      const uint32_t on_us =
          static_cast<uint32_t>(std::clamp(duty, 0.0, 1.0) * this->window_len_us_ + 0.5);
      want = (micros() - this->window_start_us_) < on_us;
    }
  }

  if (want != this->heater_on_) {
    this->heater_pin_->digital_write(want);
    this->heater_on_ = want;
  }
}

void T12Component::triggerBeep_(uint32_t duration_ms) {
  if (this->buzzer_pin_ == nullptr) return;
  this->beep_until_us_ = micros() + duration_ms * 1000;
}

void T12Component::serviceBuzzer_(uint32_t now_us) {
  if (this->buzzer_pin_ == nullptr) return;
  const bool on = static_cast<int32_t>(now_us - this->beep_until_us_) < 0;
  if (on != this->buzzer_on_) {
    this->buzzer_pin_->digital_write(on);
    this->buzzer_on_ = on;
  }
}

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------
void T12Component::onEncoderStep(int delta_steps) {
  if (delta_steps == 0) return;

  if (!this->in_setup_menu_) {
    this->scheduler_.setUserSetpoint(this->scheduler_.userSetpoint() + delta_steps * TEMP_STEP_C);
    this->scheduler_.notifyHandleMoved();
    return;
  }

  const int selected = std::clamp(static_cast<int>(this->menu_selected_) + delta_steps, 0,
                                  static_cast<int>(MENU_ITEM_COUNT) - 1);
  // Cursor follows the selection but stays inside the visible 3-row window.
  const int arrow = std::clamp(static_cast<int>(this->menu_arrow_) +
                                   (selected - static_cast<int>(this->menu_selected_)),
                               0, static_cast<int>(MENU_ROWS) - 1);
  this->menu_selected_ = static_cast<uint8_t>(selected);
  this->menu_arrow_ = static_cast<uint8_t>(arrow);
}

void T12Component::onButtonPress() {
  this->button_down_ = true;
  this->button_press_us_ = micros();
}

void T12Component::onButtonRelease() {
  if (!this->button_down_) return;
  this->button_down_ = false;
  if (micros() - this->button_press_us_ >= BUTTON_LONG_PRESS_US) {
    this->onLongPress();
  } else {
    this->onShortPress();
  }
}

void T12Component::onShortPress() {
  if (this->in_setup_menu_) {
    this->applyMenuSelection_();
    return;
  }
  this->scheduler_.toggleBoost();
  this->scheduler_.notifyHandleMoved();
  this->triggerBeep_(40);
}

void T12Component::onLongPress() {
  if (this->in_setup_menu_) {
    this->leaveSetupMenu_();
  } else {
    this->enterSetupMenu_();
  }
}

void T12Component::enterSetupMenu_() {
  this->in_setup_menu_ = true;
  this->menu_selected_ = 0;
  this->menu_arrow_ = 0;
  this->applyHeaterDuty_(0.0, micros());
  this->triggerBeep_(40);
}

void T12Component::leaveSetupMenu_() {
  this->in_setup_menu_ = false;
  this->menu_selected_ = 0;
  this->menu_arrow_ = 0;
  // No notifyHandleMoved_ here: exiting the menu must not re-arm a heater the
  // operator switched off from it.
  this->triggerBeep_(40);
}

void T12Component::applyMenuSelection_() {
  switch (this->menu_selected_) {
    case 0:
      this->setWifiEnabled(!this->wifi_enabled_);
      break;
    case 1:
      this->main_screen_type_ = (this->main_screen_type_ == 0) ? 1 : 0;
      break;
    case 2:
      if (this->scheduler_.fault() != FaultCode::None) {
        this->scheduler_.clearFault();
      } else {
        this->scheduler_.setStandbyOff(this->scheduler_.mode() != OperatingMode::Off);
      }
      break;
    case 3:
    default:
      this->leaveSetupMenu_();
      return;  // leaveSetupMenu_ beeps on its own
  }
  this->triggerBeep_(40);
}

void T12Component::setWifiEnabled(bool on) {
#ifdef USE_WIFI
  if (wifi::global_wifi_component != nullptr) {
    if (on) {
      wifi::global_wifi_component->enable();
    } else {
      wifi::global_wifi_component->disable();
    }
    this->wifi_enabled_ = on;
    ESP_LOGI(TAG, "Wi-Fi %s from the encoder", on ? "enabled" : "disabled");
    return;
  }
#endif
  this->wifi_enabled_ = false;
  ESP_LOGW(TAG, "Wi-Fi toggle unavailable: no Wi-Fi component in this build");
}

const char *T12Component::menuItemText_(uint8_t index) const {
  switch (index) {
    case 0:
      return this->wifi_enabled_ ? "Wi-Fi: ON" : "Wi-Fi: OFF";
    case 1:
      return this->main_screen_type_ == 0 ? "Screen: Big" : "Screen: Info";
    case 2:
      // While a fault is latched the press clears it, so the label must say so.
      if (this->scheduler_.fault() != FaultCode::None) return "Clear Fault";
      return this->scheduler_.mode() == OperatingMode::Off ? "Heater: OFF" : "Heater: ON";
    case 3:
    default:
      return "Return";
  }
}

void T12Component::renderDisplay(display::Display &it, font::Font *font_small, font::Font *font_med,
                                 font::Font *font_big) {
  // A 128x64 frame is ~1 KB over I2C, tens of milliseconds at 400 kHz. Park the
  // heater for the flush: software PWM cannot toggle while it blocks, and a pin
  // left on for the whole flush would deliver a large unmeasured slug of heat.
  this->display_busy_ = true;
  this->forceHeaterOff_();

  it.clear();
  if (this->in_setup_menu_) {
    this->drawSetupMenu_(it, font_small);
  } else {
    this->drawMainScreen_(it, font_small, font_med, font_big);
  }
}

void T12Component::drawMainScreen_(display::Display &it, font::Font *font_small,
                                   font::Font *font_med, font::Font *font_big) {
  char buffer[16];

  // Top row: setpoint and heater status, same positions as upstream MainScreen.
  // Fault and Off have a 0 C active setpoint, but the row must keep showing the
  // value the operator dialled in.
  const OperatingMode mode = this->scheduler_.mode();
  const double shown_sp = (mode == OperatingMode::Fault || mode == OperatingMode::Off)
                              ? this->scheduler_.userSetpoint()
                              : this->scheduler_.activeSetpoint();
  it.print(0, 0, font_small, "SET:");
  snprintf(buffer, sizeof(buffer), "%3.0f", shown_sp);
  it.print(40, 0, font_small, buffer);
  it.print(83, 0, font_small, HeaterScheduler::statusText(this->last_frame_.status));

  const bool error = this->last_frame_.status == HeaterStatusLabel::Error;

  if (this->main_screen_type_ == 1) {
    // More infos: tip name (+ Wi-Fi marker) and heater duty instead of input volts.
    std::string tip = this->tip_name_;
    if (this->wifi_enabled_) tip += " [W]";
    it.print(0, 52, font_small, tip.c_str());

    snprintf(buffer, sizeof(buffer), "%3.0f%%", this->last_frame_.effective_duty * 100.0);
    it.print(83, 52, font_small, buffer);

    if (error) {
      it.print(37, 22, font_med, "000");
    } else {
      snprintf(buffer, sizeof(buffer), "%03.0f", this->last_frame_.display_temp_c);
      it.print(37, 22, font_med, buffer);
    }
  } else {
    if (error) {
      it.print(15, 20, font_big, "000");
    } else {
      snprintf(buffer, sizeof(buffer), "%03.0f", this->last_frame_.display_temp_c);
      it.print(15, 20, font_big, buffer);
    }
  }
}

void T12Component::drawSetupMenu_(display::Display &it, font::Font *font_small) {
  it.print(0, 0, font_small, "Setup Menu");
  it.print(0, 16 * (this->menu_arrow_ + 1), font_small, ">");

  for (uint8_t i = 0; i < MENU_ROWS; ++i) {
    const int index = static_cast<int>(this->menu_selected_) -
                      static_cast<int>(this->menu_arrow_) + static_cast<int>(i);
    if (index >= 0 && index < MENU_ITEM_COUNT) {
      it.print(12, 16 * (i + 1), font_small, this->menuItemText_(static_cast<uint8_t>(index)));
    }
  }
}

float T12Component::currentTempC() const { return static_cast<float>(this->last_frame_.display_temp_c); }

float T12Component::heaterDutyPercent() const {
  return static_cast<float>(this->last_frame_.effective_duty * 100.0);
}

float T12Component::targetSetpoint() const {
  return static_cast<float>(this->scheduler_.userSetpoint());
}

void T12Component::setTargetSetpoint(float sp_c) {
  // Called from the API/web_server task: store a pending value only. Applying it
  // here could wake the station (notifyHandleMoved) or touch the scheduler from
  // another context.
  if (!std::isfinite(sp_c)) return;
  this->pending_setpoint_c_ = sp_c;
  this->pending_setpoint_valid_ = true;
}

const char *T12Component::operatingModeText() const {
  switch (this->scheduler_.mode()) {
    case OperatingMode::Boost:
      return "BOOST";
    case OperatingMode::Sleep:
      return "SLEEP";
    case OperatingMode::Off:
      return "OFF";
    case OperatingMode::Fault:
      return "FAULT";
    case OperatingMode::Active:
    default:
      return "ACTIVE";
  }
}

}  // namespace t12
}  // namespace esphome
