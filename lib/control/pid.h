#pragma once
// Pure PID controller, HAL independent (builds on host AND ESP32).
//   * parallel form: u = Kp*e + Ki*integral(e) + Kd*d(-measurement)/dt
//   * derivative on MEASUREMENT (no setpoint kick)
//   * anti-windup: conditional integration (don't integrate into saturation)
// Output is clamped to [out_min, out_max]. For the heater, output = duty 0..1.
class PID {
public:
    PID(double kp, double ki, double kd, double out_min, double out_max);

    void   setGains(double kp, double ki, double kd);
    void   setOutputLimits(double lo, double hi);
    void   setSetpoint(double sp);                   // ignored if sp is not finite
    void   reset();                                  // clears integral and p/i/d terms

    // Returns the clamped output. `new_sample` is false on control frames that
    // carry no fresh measurement (the heater is blanked for the ADC sample every
    // 5th frame at 100 Hz), so the derivative holds its last value and only the
    // proportional/integral terms advance. `sample_dt` is the interval between
    // measurements, used for the derivative when it differs from the frame dt.
    double update(double measurement, double dt,
                  bool new_sample = true, double sample_dt = 0.0);

    // telemetry / introspection
    double kp() const { return kp_; }
    double ki() const { return ki_; }
    double kd() const { return kd_; }
    double setpoint() const { return sp_; }
    double pTerm() const { return p_; }
    double iTerm() const { return i_; }
    double dTerm() const { return d_; }
    double integral() const { return integral_; }

private:
    double kp_, ki_, kd_;
    double out_min_, out_max_;
    double sp_        = 0.0;
    double integral_  = 0.0;
    double prev_meas_ = 0.0;
    bool   has_prev_  = false;
    double p_ = 0.0, i_ = 0.0, d_ = 0.0;
};
