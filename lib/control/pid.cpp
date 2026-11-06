#include "pid.h"
#include <algorithm>
#include <cmath>

PID::PID(double kp, double ki, double kd, double out_min, double out_max)
    : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max) {}

void PID::setGains(double kp, double ki, double kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
void PID::setOutputLimits(double lo, double hi) { out_min_ = lo; out_max_ = hi; }

void PID::setSetpoint(double sp) {
    // A non-finite setpoint would make every error term NaN and poison the
    // integral permanently.
    if (!std::isfinite(sp)) return;
    if (sp != sp_) {
        // A setpoint move restarts derivative history so the first sample after
        // the step is not differenced against one taken under the old target.
        // No kick to suppress: the derivative is on measurement, not error.
        has_prev_ = false;
        d_        = 0.0;
    }
    sp_ = sp;
}

void PID::reset() {
    integral_  = 0.0;
    prev_meas_ = 0.0;
    has_prev_  = false;
    p_         = 0.0;
    i_         = 0.0;
    d_         = 0.0;
}

double PID::update(double meas, double dt, bool new_sample, double sample_dt) {
    if (dt <= 0.0) return 0.0;

    const double error = sp_ - meas;

    // Proportional
    p_ = kp_ * error;

    // Derivative on measurement (negative because d(error)/dt = -d(meas)/dt).
    // Only refreshed when a new measurement actually arrived: differencing the
    // same reading across blanked frames would report zero slope and then a
    // spike on the next sample.
    if (new_sample) {
        const double d_dt = (sample_dt > 0.0) ? sample_dt : dt;
        const double deriv = has_prev_ ? -(meas - prev_meas_) / d_dt : 0.0;
        has_prev_  = true;
        prev_meas_ = meas;
        d_ = kd_ * deriv;
    }
    // else: d_ keeps the value from the most recent sample (0.0 before the first)

    // Tentative integral step
    const double integral_candidate = integral_ + error * dt;
    const double out_unclamped = p_ + ki_ * integral_candidate + d_;
    double out = std::clamp(out_unclamped, out_min_, out_max_);

    // ---- conditional integration (anti-windup) ----
    // Freeze the integral only when saturated AND the error pushes further into
    // saturation. Otherwise accept the step (this also lets it unwind out of a limit).
    const bool saturated = (out != out_unclamped);
    const bool pushing_further =
        saturated && ((out_unclamped > out_max_ && error > 0.0) ||
                      (out_unclamped < out_min_ && error < 0.0));
    if (!pushing_further) {
        integral_ = integral_candidate;
        i_ = ki_ * integral_;
        out = std::clamp(p_ + i_ + d_, out_min_, out_max_);
    }
    // else: integral_ and i_ keep their previous values

    return out;
}
