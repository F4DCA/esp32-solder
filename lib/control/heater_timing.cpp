#include "heater_timing.h"

#include <algorithm>
#include <cmath>

namespace {
// Slack for the elapsed-seconds timers: 30000 accumulated 0.01 s frames must
// still trip the 300 s timeout on the frame they should, not one frame later
// from floating-point round-off. A microsecond is far below any real cadence.
constexpr double kTimerEps = 1e-6;
}  // namespace

// ---------------------------------------------------------------------------
// Front-end calibration
// ---------------------------------------------------------------------------
double rawToTempC(double raw_reading, const FrontEndConfig& cfg) {
    if (cfg.mode == FrontEndMode::DirectTcLinear) {
        const double uv_per_c =
            (std::abs(cfg.tc_uv_per_c) < 1e-6) ? 21.0 : cfg.tc_uv_per_c;
        return cfg.cjc_temp_c + raw_reading / uv_per_c;
    }

    // Piecewise-linear lookup on the three calibration points, the same shape as
    // the upstream calculateTemp() map(0..200), map(200..280), map(280..360).
    // Degenerate tables are pushed apart instead of dividing by zero.
    const double cal_200 = std::max(cfg.cal_200, 1.0);
    const double cal_280 = std::max(cfg.cal_280, cal_200 + 1.0);
    const double cal_360 = std::max(cfg.cal_360, cal_280 + 1.0);
    const double ambient = cfg.cal_ambient_c;

    if (raw_reading <= 0.0) return ambient;
    if (raw_reading < cal_200)
        return ambient + raw_reading * (200.0 - ambient) / cal_200;
    if (raw_reading < cal_280)
        return 200.0 + (raw_reading - cal_200) * 80.0 / (cal_280 - cal_200);
    if (raw_reading <= cal_360)
        return 280.0 + (raw_reading - cal_280) * 80.0 / (cal_360 - cal_280);

    // Above the top point: extrapolate on the 280..360 C slope. The table tops
    // out at 360 C but the iron works at 350 C and must still be able to report
    // a tip that has gone too hot.
    return 360.0 + (raw_reading - cal_360) * 80.0 / (cal_360 - cal_280);
}

// ---------------------------------------------------------------------------
// HeaterScheduler
// ---------------------------------------------------------------------------
HeaterScheduler::HeaterScheduler(const TimingConfig& cfg) : cfg_(cfg) {
    if (cfg_.frame_dt_s <= 0.0) cfg_.frame_dt_s = 0.010;
    if (cfg_.sample_stride < 1) cfg_.sample_stride = 1;
    reset();
}

void HeaterScheduler::reset(double initial_temp_c) {
    user_setpoint_c_ = std::clamp(cfg_.default_setpoint_c,
                                  cfg_.min_setpoint_c, cfg_.max_setpoint_c);
    measured_temp_c_ = initial_temp_c;
    display_temp_c_  = initial_temp_c;
    idle_elapsed_s_  = 0.0;
    boost_elapsed_s_ = 0.0;
    frame_index_     = 0;
    mode_            = OperatingMode::Active;
    fault_           = FaultCode::None;
    last_sample_c_   = initial_temp_c;
    last_sample_valid_ = true;
    is_worky_        = false;
    beep_if_worky_   = false;
}

void HeaterScheduler::setUserSetpoint(double sp_c) {
    // A NaN slips through std::clamp and then poisons the PID error and integral.
    if (!std::isfinite(sp_c)) return;
    user_setpoint_c_ = std::clamp(sp_c, cfg_.min_setpoint_c, cfg_.max_setpoint_c);
}

double HeaterScheduler::activeSetpoint() const {
    switch (mode_) {
        case OperatingMode::Fault:
        case OperatingMode::Off:
            return 0.0;
        case OperatingMode::Sleep:
            return cfg_.sleep_temp_c;
        case OperatingMode::Boost: {
            // Never park the boosted setpoint into the tip-removal trip point.
            return std::min(user_setpoint_c_ + cfg_.boost_delta_c,
                            cfg_.max_cutoff_temp_c - 10.0);
        }
        case OperatingMode::Active:
        default:
            return user_setpoint_c_;
    }
}

void HeaterScheduler::notifyHandleMoved() {
    idle_elapsed_s_ = 0.0;
    if (mode_ == OperatingMode::Sleep || mode_ == OperatingMode::Off) {
        mode_ = OperatingMode::Active;
    }
}

void HeaterScheduler::toggleBoost() {
    if (mode_ == OperatingMode::Boost) {
        mode_            = OperatingMode::Active;
        boost_elapsed_s_ = 0.0;
    } else if (mode_ == OperatingMode::Active) {
        mode_            = OperatingMode::Boost;
        boost_elapsed_s_ = 0.0;
    }
}

void HeaterScheduler::setStandbyOff(bool off) {
    if (off) {
        mode_     = OperatingMode::Off;
        is_worky_ = false;
    } else if (mode_ == OperatingMode::Off) {
        mode_           = OperatingMode::Active;
        idle_elapsed_s_ = 0.0;
    }
}

void HeaterScheduler::clearFault() {
    if (fault_ == FaultCode::None) return;
    // Only drop the latch on fresh data that sits inside the sane band: a tip
    // that is still missing, still shorted, or a bus that is still dead must not
    // re-enable the heater.
    const bool sane = last_sample_valid_ && std::isfinite(last_sample_c_) &&
                      last_sample_c_ <= cfg_.max_cutoff_temp_c &&
                      last_sample_c_ >= cfg_.min_sane_temp_c;
    if (!sane) return;

    fault_           = FaultCode::None;
    mode_            = OperatingMode::Active;
    idle_elapsed_s_  = 0.0;
    boost_elapsed_s_ = 0.0;
}

bool HeaterScheduler::isSampleFrame() const {
    return (cfg_.sample_stride <= 1) ||
           ((frame_index_ % static_cast<uint32_t>(cfg_.sample_stride)) == 0u);
}

double HeaterScheduler::blankFraction() const {
    return (cfg_.settle_us + cfg_.conv_us) * 1e-6 / cfg_.frame_dt_s;
}

double HeaterScheduler::sampleFrameMaxDuty() const {
    return std::clamp(1.0 - blankFraction(), 0.0, 1.0);
}

double HeaterScheduler::cycleAverageMaxDuty() const {
    const double stride = static_cast<double>(std::max(1, cfg_.sample_stride));
    return (sampleFrameMaxDuty() + (stride - 1.0)) / stride;
}

const char* HeaterScheduler::statusText(HeaterStatusLabel s) {
    switch (s) {
        case HeaterStatusLabel::Error: return "ERROR";
        case HeaterStatusLabel::Off:   return "  OFF";
        case HeaterStatusLabel::Sleep: return "SLEEP";
        case HeaterStatusLabel::Boost: return "BOOST";
        case HeaterStatusLabel::Worky: return "WORKY";
        case HeaterStatusLabel::Heat:  return " HEAT";
        case HeaterStatusLabel::Hold:
        default:                       return " HOLD";
    }
}

FrameResult HeaterScheduler::stepFrame(PID& pid, double new_temp_c, bool sample_valid,
                                       double elapsed_s) {
    FrameResult fr;
    fr.frame_index  = frame_index_;
    fr.sample_frame = isSampleFrame();

    // ---- measurement: only on sample frames, and only if it is believable ----
    if (fr.sample_frame) {
        last_sample_valid_ = sample_valid && std::isfinite(new_temp_c);
        if (last_sample_valid_) last_sample_c_ = new_temp_c;

        if (!sample_valid || !std::isfinite(new_temp_c) ||
            new_temp_c < cfg_.min_sane_temp_c) {
            // A shorted front end reads far below any real tip; treat it exactly
            // like a dead bus, because either way there is no measurement the
            // loop can heat against.
            fault_ = FaultCode::SensorReadError;
            mode_  = OperatingMode::Fault;
        } else if (new_temp_c > cfg_.disconnect_temp_c) {
            fault_ = FaultCode::TipDisconnected;
            mode_  = OperatingMode::Fault;
        } else if (new_temp_c > cfg_.max_cutoff_temp_c) {
            fault_ = FaultCode::OverTemperature;
            mode_  = OperatingMode::Fault;
        } else {
            measured_temp_c_ = new_temp_c;

            // Upstream display stabilisation: hold the shown value until it has
            // drifted more than 5 C from the reading, then snap it over; a shown
            // value within 1 C of the setpoint is drawn as the setpoint exactly.
            const double sp = activeSetpoint();
            if ((display_temp_c_ != sp) ||
                (std::abs(display_temp_c_ - measured_temp_c_) > 5.0)) {
                display_temp_c_ = measured_temp_c_;
            }
            if (std::abs(display_temp_c_ - sp) <= 1.0) display_temp_c_ = sp;
        }
    }

    updateOutput(pid, fr.sample_frame, (elapsed_s > 0.0) ? elapsed_s : cfg_.frame_dt_s, fr);
    finalizeFrame(fr);
    return fr;
}

FrameResult HeaterScheduler::stepFrameHeld(PID& pid, double elapsed_s) {
    FrameResult fr;
    fr.frame_index = frame_index_;

    updateOutput(pid, false, (elapsed_s > 0.0) ? elapsed_s : cfg_.frame_dt_s, fr);
    finalizeFrame(fr);
    return fr;
}

// ---------------------------------------------------------------------------
// Shared frame tail: timers, fault priority, control output and label
// ---------------------------------------------------------------------------
void HeaterScheduler::advanceTimers(double dt_s, FrameResult& fr) {
    if (mode_ == OperatingMode::Boost) {
        boost_elapsed_s_ += dt_s;
        if (cfg_.boost_duration_s > 0.0 &&
            boost_elapsed_s_ >= cfg_.boost_duration_s - kTimerEps) {
            mode_            = OperatingMode::Active;
            boost_elapsed_s_ = 0.0;
            beep_if_worky_   = true;  // beep again once the working point is back
            fr.beep_event    = true;
        }
    } else if (mode_ == OperatingMode::Active || mode_ == OperatingMode::Sleep) {
        idle_elapsed_s_ += dt_s;
        if (mode_ == OperatingMode::Active && cfg_.time_to_sleep_s > 0.0 &&
            idle_elapsed_s_ >= cfg_.time_to_sleep_s - kTimerEps) {
            mode_         = OperatingMode::Sleep;
            fr.beep_event = true;
        }
        if (cfg_.time_to_off_s > 0.0 &&
            idle_elapsed_s_ >= cfg_.time_to_off_s - kTimerEps &&
            mode_ != OperatingMode::Off) {
            mode_         = OperatingMode::Off;
            is_worky_     = false;
            fr.beep_event = true;
        }
    }
}

void HeaterScheduler::updateOutput(PID& pid, bool new_sample, double dt_s,
                                   FrameResult& fr) {
    // A latched fault outranks every mode request: setStandbyOff(false) or a
    // handle move must never re-arm a station the loop has tripped.
    if (fault_ != FaultCode::None) mode_ = OperatingMode::Fault;

    advanceTimers(dt_s, fr);

    const double active_sp = activeSetpoint();
    fr.active_setpoint_c   = active_sp;

    if (mode_ == OperatingMode::Fault || mode_ == OperatingMode::Off) {
        pid.reset();
        fr.pid_duty            = 0.0;
        fr.effective_duty      = 0.0;
        fr.post_blank_pwm_duty = 0.0;
        is_worky_              = false;
        return;
    }

    pid.setSetpoint(active_sp);
    const double raw_duty = pid.update(measured_temp_c_, cfg_.frame_dt_s, new_sample,
                                       cfg_.frame_dt_s * cfg_.sample_stride);
    fr.pid_duty = raw_duty;

    if (fr.sample_frame) {
        // The heater was off for the whole blanking window, so this frame can
        // only deliver what fits after it. Scaling the post-blank duty by the
        // remaining fraction keeps the delivered energy equal to the clipped
        // command: 0.75 * (d / 0.75) for 7.5 ms == d for 10 ms.
        const double max_duty  = sampleFrameMaxDuty();
        fr.effective_duty      = std::min(raw_duty, max_duty);
        fr.post_blank_pwm_duty = (max_duty > 0.0)
                                     ? std::clamp(fr.effective_duty / max_duty, 0.0, 1.0)
                                     : 0.0;
    } else {
        fr.effective_duty      = raw_duty;
        fr.post_blank_pwm_duty = raw_duty;
    }

    if (std::abs(active_sp - measured_temp_c_) < cfg_.worky_band_c) {
        if (!is_worky_ && beep_if_worky_) {
            fr.beep_event  = true;
            beep_if_worky_ = false;
        }
        is_worky_ = true;
    } else {
        is_worky_ = false;
    }
}

void HeaterScheduler::finalizeFrame(FrameResult& fr) {
    // ---- display status label (upstream MainScreen precedence) ----
    if (mode_ == OperatingMode::Fault) {
        fr.status = HeaterStatusLabel::Error;
    } else if (mode_ == OperatingMode::Off) {
        fr.status = HeaterStatusLabel::Off;
    } else if (mode_ == OperatingMode::Sleep) {
        fr.status = HeaterStatusLabel::Sleep;
    } else if (mode_ == OperatingMode::Boost) {
        fr.status = HeaterStatusLabel::Boost;
    } else if (is_worky_) {
        fr.status = HeaterStatusLabel::Worky;
    } else if (fr.effective_duty > 0.30) {
        fr.status = HeaterStatusLabel::Heat;
    } else {
        fr.status = HeaterStatusLabel::Hold;
    }

    fr.measured_temp_c = measured_temp_c_;
    fr.display_temp_c  = display_temp_c_;
    fr.mode            = mode_;
    fr.fault           = fault_;

    ++frame_index_;
}
