#include "pid.h"
#include <algorithm>

PID::PID(double kp, double ki, double kd, double out_min, double out_max)
    : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max) {}

void PID::setGains(double kp, double ki, double kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
void PID::setOutputLimits(double lo, double hi) { out_min_ = lo; out_max_ = hi; }

void PID::setSetpoint(double sp) {
    if (sp != sp_) has_prev_ = false;   // SP change: drop derivative memory (no kick)
    sp_ = sp;
}

void PID::reset() { integral_ = 0.0; prev_meas_ = 0.0; has_prev_ = false; }

double PID::update(double meas, double dt) {
    if (dt <= 0.0) return 0.0;

    const double error = sp_ - meas;

    // Proportional
    p_ = kp_ * error;

    // Derivative on measurement (negative because d(error)/dt = -d(meas)/dt)
    double deriv = 0.0;
    if (has_prev_) deriv = -(meas - prev_meas_) / dt;
    has_prev_ = true;
    d_ = kd_ * deriv;

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

    prev_meas_ = meas;
    return out;
}
