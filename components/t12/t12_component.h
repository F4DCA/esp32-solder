#pragma once
// ESPHome shell for the T12 station: owns the 100 Hz frame, the heater-off
// thermocouple interlock, the ADS1115 read, the encoder/menu state machine and
// the OLED layout. Control mathematics live in lib/control, which stays
// HAL-independent so the native simulation gates this code.
#include <string>

#include "esphome/components/display/display.h"
#include "esphome/components/font/font.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "heater_timing.h"
#include "pid.h"

namespace esphome {
namespace t12 {

class T12Component : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  // ---- configuration, wired from components/t12/__init__.py ----
  void set_heater_pin(GPIOPin *pin) { this->heater_pin_ = pin; }
  void set_shake_pin(GPIOPin *pin) { this->shake_pin_ = pin; }
  void set_buzzer_pin(GPIOPin *pin) { this->buzzer_pin_ = pin; }
  void set_frontend_mode(uint8_t mode) {
    this->frontend_cfg_.mode = static_cast<FrontEndMode>(mode);
  }
  void set_cal_200(double v) { this->frontend_cfg_.cal_200 = v; }
  void set_cal_280(double v) { this->frontend_cfg_.cal_280 = v; }
  void set_cal_360(double v) { this->frontend_cfg_.cal_360 = v; }
  void set_cal_ambient(double v) { this->frontend_cfg_.cal_ambient_c = v; }
  void set_tc_uv_per_c(double v) { this->frontend_cfg_.tc_uv_per_c = v; }
  void set_cjc_temp(double v) { this->frontend_cfg_.cjc_temp_c = v; }
  void set_ema_alpha(double a) { this->frontend_cfg_.ema_alpha = a; }
  void set_gains(double kp, double ki, double kd) {
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;
  }
  void set_default_setpoint(double sp) { this->default_setpoint_c_ = sp; }
  void set_tip_name(const std::string &name) { this->tip_name_ = name; }
  void set_csv_telemetry(bool on) { this->csv_telemetry_ = on; }

  // ---- UI events, wired from firmware/t12.yaml ----
  void onEncoderStep(int delta_steps);
  void onButtonPress();
  void onButtonRelease();
  void onShortPress();
  void onLongPress();
  void renderDisplay(display::Display &it, font::Font *font_small, font::Font *font_med,
                     font::Font *font_big);

  // ---- observers for Home Assistant / web_server ----
  float currentTempC() const;
  float heaterDutyPercent() const;
  float targetSetpoint() const;
  void setTargetSetpoint(float sp_c);
  const char *operatingModeText() const;
  bool wifiEnabled() const { return this->wifi_enabled_; }
  void setWifiEnabled(bool on);

 protected:
  enum class SamplePhase : uint8_t { Idle, WaitSettle, WaitConversion };

  void serviceFrame_(uint32_t now_us);
  void serviceSample_(uint32_t now_us);
  void finishSample_(bool sample_valid, double temp_c);
  bool startAdsConversion_();
  bool readAds1115_(double *temp_c);
  void applyHeaterDuty_(double duty, uint32_t now_us);
  void updateHeaterOutput_();
  void forceHeaterOff_();
  void triggerBeep_(uint32_t duration_ms);
  void serviceBuzzer_(uint32_t now_us);
  void enterSetupMenu_();
  void leaveSetupMenu_();
  void applyMenuSelection_();
  void drawMainScreen_(display::Display &it, font::Font *font_small, font::Font *font_med,
                       font::Font *font_big);
  void drawSetupMenu_(display::Display &it, font::Font *font_small);
  const char *menuItemText_(uint8_t index) const;

  GPIOPin *heater_pin_{nullptr};
  GPIOPin *shake_pin_{nullptr};
  GPIOPin *buzzer_pin_{nullptr};

  FrontEndConfig frontend_cfg_{};
  HeaterScheduler scheduler_{};  // frame cadence, interlock ceiling, modes, faults
  PID pid_{0.0048, 0.0036, 0.0, 0.0, 1.0};

  std::string tip_name_{"BC1.5"};
  double kp_{0.0048};
  double ki_{0.0036};
  double kd_{0.0};
  double default_setpoint_c_{320.0};

  bool csv_telemetry_{true};
  bool in_setup_menu_{false};
  bool wifi_enabled_{false};
  uint8_t menu_selected_{0};
  uint8_t menu_arrow_{0};
  uint8_t main_screen_type_{0};  // 0 = big numbers, 1 = more infos

  bool display_busy_{false};      // set while the OLED lambda/flush runs

  // Setpoint pushed over the network or REST: validated and stored here, applied
  // at the next loop() pass so it can never wake the scheduler or bump the idle
  // timer from a callback thread.
  volatile float pending_setpoint_c_{0.0f};
  volatile bool pending_setpoint_valid_{false};

  SamplePhase sample_phase_{SamplePhase::Idle};
  uint32_t frame_us_{10000};
  uint32_t settle_us_{950};
  uint32_t conv_us_{1550};
  uint32_t last_frame_us_{0};
  uint32_t frame_start_us_{0};
  uint32_t frame_prev_start_us_{0};  // start of the frame that just ended
  uint32_t blank_start_us_{0};
  uint32_t conv_start_us_{0};
  uint32_t button_press_us_{0};
  bool button_down_{false};
  uint32_t beep_until_us_{0};
  bool buzzer_on_{false};
  uint32_t sample_count_{0};  // completed measurements, drives the 5 Hz CSV log

  double current_pwm_duty_{0.0};   // duty to deliver over the current window
  uint32_t window_start_us_{0};    // when that window opened
  uint32_t window_len_us_{0};      // how long it lasts (rest of the frame)
  bool in_blanking_window_{false};
  bool heater_on_{false};
  double ema_temp_c_{26.0};
  bool ema_valid_{false};
  bool last_shake_{false};
  FrameResult last_frame_{};
};

}  // namespace t12
}  // namespace esphome
