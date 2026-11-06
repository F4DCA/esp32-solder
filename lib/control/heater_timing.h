#pragma once
// Heater timing interlock, analog front-end calibration and station state machine.
// HAL independent (no Arduino/ESP-IDF headers): the native simulation and the
// ESPHome component compile this exact code.
//
// The T12 cartridge shares its two leads between heater and thermocouple, so the
// FET must be off while the ADC reads the tip. The cost is time: a measurement
// costs ~2.5 ms (950 us settle + 1.16 ms ADS1115 conversion + I2C overhead) out
// of every frame it blanks. Sampling every 5th frame of a 100 Hz loop leaves a
// 0.75 ceiling on the sampling frame and 0.95 over the cycle, which keeps the
// achievable temperature above the 350 C working point (T_max = 26 + 414*d).
#include <cstdint>

#include "pid.h"

// ---------------------------------------------------------------------------
// Analog front end: raw reading -> tip temperature
// ---------------------------------------------------------------------------
enum class FrontEndMode : uint8_t {
    OpAmp3Point = 0,    // op-amp (OPA2330/MCP6002) + 3-point CalTemp lookup
    DirectTcLinear = 1  // T12 thermocouple straight into the ADS1115 differential input
};

struct FrontEndConfig {
    FrontEndMode mode    = FrontEndMode::OpAmp3Point;
    double cal_200       = 216.0;  // ADC reading at 200 C (upstream TEMP200)
    double cal_280       = 308.0;  // ADC reading at 280 C (upstream TEMP280)
    double cal_360       = 390.0;  // ADC reading at 360 C (upstream TEMP360)
    double cal_ambient_c = 21.0;   // baseline for the 0..cal_200 segment
    double tc_uv_per_c   = 21.0;   // T12 Seebeck coefficient, uV/C (DirectTcLinear)
    double cjc_temp_c    = 26.0;   // cold-junction temperature, C (DirectTcLinear)
    double ema_alpha     = 1.0;    // measurement EMA factor (1.0 = unsmoothed)
};

// raw_reading is an ADC count for OpAmp3Point and a differential voltage in uV
// for DirectTcLinear. Returns NaN if the reading is NaN so the caller faults.
double rawToTempC(double raw_reading, const FrontEndConfig& cfg);

// ---------------------------------------------------------------------------
// Timing, operating modes and faults
// ---------------------------------------------------------------------------
struct TimingConfig {
    double frame_dt_s         = 0.010;  // 100 Hz control frame (10 ms)
    int    sample_stride      = 5;      // sample every 5th frame (20 Hz = 50 ms)
    double settle_us          = 950.0;  // heater-off settle delay before the ADC read
    double conv_us            = 1550.0; // ADS1115 @860 SPS (1.16 ms) + I2C read overhead
    double min_setpoint_c     = 150.0;  // TEMP_MIN
    double max_setpoint_c     = 400.0;  // TEMP_MAX
    double default_setpoint_c = 320.0;  // TEMP_DEFAULT
    double sleep_temp_c       = 150.0;  // TEMP_SLEEP
    double boost_delta_c      = 50.0;   // TEMP_BOOST
    double max_cutoff_temp_c  = 450.0;  // safety over-temperature cutoff
    double disconnect_temp_c  = 500.0;  // tip removed (upstream "ShowTemp > 500")
    double min_sane_temp_c    = -50.0;  // below this the front end is broken, not cold
    double worky_band_c       = 5.0;    // |setpoint - temp| < 5 C -> WORKY
    double time_to_sleep_s    = 300.0;  // idle -> Sleep (0 = disabled)
    double time_to_off_s      = 900.0;  // idle -> Off   (0 = disabled)
    double boost_duration_s   = 40.0;   // boost timer (0 = disabled)
};

enum class OperatingMode : uint8_t {
    Active = 0,
    Boost  = 1,
    Sleep  = 2,
    Off    = 3,
    Fault  = 4
};

enum class FaultCode : uint8_t {
    None            = 0,
    TipDisconnected = 1,
    OverTemperature = 2,
    SensorReadError = 3
};

enum class HeaterStatusLabel : uint8_t {
    Error = 0,  // "ERROR"
    Off   = 1,  // "  OFF"
    Sleep = 2,  // "SLEEP"
    Boost = 3,  // "BOOST"
    Worky = 4,  // "WORKY"
    Heat  = 5,  // " HEAT"
    Hold  = 6   // " HOLD"
};

struct FrameResult {
    uint32_t          frame_index         = 0;
    bool              sample_frame        = false;
    double            active_setpoint_c   = 0.0;
    double            measured_temp_c     = 0.0;
    double            display_temp_c      = 0.0;
    double            pid_duty            = 0.0;  // PID output [0..1]
    double            effective_duty      = 0.0;  // delivered over the frame, blanking included
    double            post_blank_pwm_duty = 0.0;  // duty to apply after the blanking window
    OperatingMode     mode                = OperatingMode::Active;
    FaultCode         fault               = FaultCode::None;
    HeaterStatusLabel status              = HeaterStatusLabel::Off;
    bool              beep_event          = false;
};

// ---------------------------------------------------------------------------
// Frame scheduler: owns the sample cadence, the duty ceiling, and the modes
// ---------------------------------------------------------------------------
class HeaterScheduler {
public:
    explicit HeaterScheduler(const TimingConfig& cfg = TimingConfig{});

    void   reset(double initial_temp_c = 26.0);
    void   setUserSetpoint(double sp_c);          // clamped to [min, max]
    double userSetpoint() const { return user_setpoint_c_; }
    double activeSetpoint() const;                // setpoint of the current mode

    void notifyHandleMoved();                     // wake from Sleep/Off, restart idle timer
    void toggleBoost();                           // short press
    void setStandbyOff(bool off);                 // menu "Heater" item
    void clearFault();                            // drop a latch when the sensor reads sane again

    bool   isSampleFrame() const;
    double blankFraction() const;                 // off-window / frame (0.25)
    double sampleFrameMaxDuty() const;            // 1 - blankFraction (0.75)
    double cycleAverageMaxDuty() const;           // 0.95 at 5x stride

    // One control frame. A measurement is consumed only every `sample_stride`
    // frames; `elapsed_s` is the wall time since the previous call (<= 0 falls
    // back to cfg_.frame_dt_s).
    FrameResult stepFrame(PID& pid, double new_temp_c, bool sample_valid = true,
                          double elapsed_s = 0.0);

    // Menu path: advances the timers and recomputes mode/status/duty from the
    // last measurement without consuming a new one, so an open menu cannot
    // stall Sleep/Off or leave a stale duty latched.
    FrameResult stepFrameHeld(PID& pid, double elapsed_s = 0.0);

    OperatingMode   mode() const { return mode_; }
    FaultCode       fault() const { return fault_; }
    double          lastMeasuredTemp() const { return measured_temp_c_; }
    double          displayTemp() const { return display_temp_c_; }
    bool            isWorky() const { return is_worky_; }
    const TimingConfig& config() const { return cfg_; }
    static const char* statusText(HeaterStatusLabel s);

private:
    void advanceTimers(double dt_s, FrameResult& fr);
    void updateOutput(PID& pid, bool new_sample, double dt_s, FrameResult& fr);
    void finalizeFrame(FrameResult& fr);

    TimingConfig cfg_;
    double user_setpoint_c_ = 0.0;
    double measured_temp_c_ = 26.0;
    double display_temp_c_  = 26.0;
    // Newest reading off the ADC, kept even when it is out of range so a latched
    // fault can be validated against fresh data instead of the last good value.
    double last_sample_c_   = 26.0;
    // Timers accumulate seconds, not frames: the ESPHome component reports the
    // real time between frames, which is not always one nominal period.
    double idle_elapsed_s_  = 0.0;
    double boost_elapsed_s_ = 0.0;
    uint32_t frame_index_   = 0;
    OperatingMode mode_     = OperatingMode::Active;
    FaultCode fault_        = FaultCode::None;
    bool last_sample_valid_ = true;
    bool is_worky_          = false;
    bool beep_if_worky_     = false;
};
