// Host simulation harness: runs the SAME control code (lib/control) that runs on
// the ESP32, against the thermal plant, and writes a step-response CSV.
//
//   output: step_response.csv   (t_ms,setpoint,temp,duty,p,i,d)
//   build : pio run                     (env:native)
//   run   : .pio/build/native/program    [--self-test]
//
// The loop is the real one: 100 Hz frames, a heater-off ADC sample every 5th
// frame, and the duty ceiling that sampling window imposes.
#include <cmath>
#include <cstdio>
#include <cstring>

#include "heater_timing.h"
#include "pid.h"
#include "plant.h"

namespace {

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_failures;
}

void checkNear(double got, double want, double tol, const char* what) {
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("%s  %-58s got %9.3f  want %9.3f\n",
                ok ? "PASS" : "FAIL", what, got, want);
    if (!ok) ++g_failures;
}

int runSelfTests() {
    std::printf("-- front-end calibration --\n");

    // Default upstream CalTemp table: ADC 216/308/390 <-> 200/280/360 C.
    FrontEndConfig fe;  // OpAmp3Point, table 216/308/390, ambient 21 C
    checkNear(rawToTempC(216.0, fe), 200.0, 1e-9, "OpAmp3Point: ADC 216 -> 200 C");
    checkNear(rawToTempC(308.0, fe), 280.0, 1e-9, "OpAmp3Point: ADC 308 -> 280 C");
    checkNear(rawToTempC(390.0, fe), 360.0, 1e-9, "OpAmp3Point: ADC 390 -> 360 C");
    checkNear(rawToTempC(0.0, fe), 21.0, 1e-9, "OpAmp3Point: ADC 0 -> ambient");
    checkNear(rawToTempC(600.0, fe), 360.0 + 210.0 * 80.0 / 82.0, 1e-9,
              "OpAmp3Point: above 360 C extrapolates on the 280..360 slope");
    checkNear(rawToTempC(262.0, fe), 240.0, 1e-9, "OpAmp3Point: 200..280 midpoint");
    check(std::isfinite(rawToTempC(50.0, fe)), "OpAmp3Point: low reading stays finite");

    FrontEndConfig degenerate;  // zeroed table must not divide by zero
    degenerate.cal_200 = 0.0;
    degenerate.cal_280 = 0.0;
    degenerate.cal_360 = 0.0;
    check(std::isfinite(rawToTempC(100.0, degenerate)) &&
              std::isfinite(rawToTempC(500.0, degenerate)),
          "OpAmp3Point: degenerate table stays finite");

    FrontEndConfig tc;
    tc.mode        = FrontEndMode::DirectTcLinear;
    tc.tc_uv_per_c = 21.0;
    tc.cjc_temp_c  = 26.0;
    checkNear(rawToTempC(6804.0, tc), 350.0, 1e-9,
              "DirectTcLinear: 6804 uV @21 uV/C, cjc 26 C -> 350 C");
    checkNear(rawToTempC(0.0, tc), 26.0, 1e-9, "DirectTcLinear: 0 uV -> cold junction");

    FrontEndConfig tc_zero = tc;
    tc_zero.tc_uv_per_c = 0.0;
    checkNear(rawToTempC(21.0, tc_zero), 27.0, 1e-9,
              "DirectTcLinear: zero coefficient falls back to 21 uV/C");

    std::printf("-- duty ceiling --\n");
    {
        TimingConfig cfg;
        HeaterScheduler s(cfg);
        checkNear(s.blankFraction(), 0.25, 1e-12, "blanking window is 25 % of the frame");
        checkNear(s.sampleFrameMaxDuty(), 0.75, 1e-12, "sample frame tops out at 0.75");
        checkNear(s.cycleAverageMaxDuty(), 0.95, 1e-12, "cycle average tops out at 0.95");

        PID pid(0.0048, 0.0036, 0.0, 0.0, 1.0);
        s.reset(26.0);
        s.setUserSetpoint(350.0);
        bool ceiling_ok = true, post_blank_ok = true;
        double max_sample = 0.0, max_other = 0.0;
        for (int n = 0; n < 500; ++n) {
            const FrameResult fr = s.stepFrame(pid, 26.0, true);  // cold: saturated
            if (fr.sample_frame) {
                max_sample = std::max(max_sample, fr.effective_duty);
                if (fr.effective_duty > 0.75 + 1e-12) ceiling_ok = false;
                if (std::fabs(fr.post_blank_pwm_duty - fr.effective_duty / 0.75) > 1e-12)
                    post_blank_ok = false;
            } else {
                max_other = std::max(max_other, fr.effective_duty);
            }
        }
        check(ceiling_ok, "sample frames never exceed the 0.75 ceiling while saturated");
        check(post_blank_ok, "post-blank duty restores the clipped frame energy");
        checkNear(max_sample, 0.75, 1e-12, "saturated sample frame delivers 0.75");
        checkNear(max_other, 1.0, 1e-12, "saturated non-sample frame delivers 1.0");
    }

    std::printf("-- faults --\n");
    {
        TimingConfig cfg;
        cfg.time_to_sleep_s = 0.0;
        cfg.time_to_off_s   = 0.0;
        HeaterScheduler s(cfg);
        PID pid(0.0048, 0.0036, 0.0, 0.0, 1.0);
        s.reset(26.0);
        s.setUserSetpoint(350.0);

        FrameResult fr;
        for (int n = 0; n < 5; ++n) fr = s.stepFrame(pid, 550.0, true);  // tip pulled
        checkNear(fr.effective_duty, 0.0, 1e-12, "tip disconnect forces effective duty 0");
        check(fr.mode == OperatingMode::Fault, "tip disconnect latches Fault");
        check(fr.fault == FaultCode::TipDisconnected, "fault code is TipDisconnected");
        check(std::strcmp(HeaterScheduler::statusText(fr.status), "ERROR") == 0,
              "status label is ERROR");

        s.clearFault();
        check(s.fault() == FaultCode::TipDisconnected,
              "clearFault keeps the latch while the reading is out of range");

        HeaterScheduler s2(cfg);
        PID pid2(0.0048, 0.0036, 0.0, 0.0, 1.0);
        s2.reset(26.0);
        s2.setUserSetpoint(350.0);
        FrameResult f2;
        for (int n = 0; n < 5; ++n) f2 = s2.stepFrame(pid2, 26.0, false);
        check(f2.fault == FaultCode::SensorReadError, "I2C failure latches SensorReadError");
        checkNear(f2.effective_duty, 0.0, 1e-12, "sensor error forces effective duty 0");

        HeaterScheduler s3(cfg);
        PID pid3(0.0048, 0.0036, 0.0, 0.0, 1.0);
        s3.reset(26.0);
        s3.setUserSetpoint(350.0);
        FrameResult f3;
        for (int n = 0; n < 5; ++n) f3 = s3.stepFrame(pid3, 470.0, true);
        check(f3.fault == FaultCode::OverTemperature, "> 450 C latches OverTemperature");
    }

    std::printf("-- modes --\n");
    {
        TimingConfig cfg;
        cfg.time_to_sleep_s  = 300.0;
        cfg.time_to_off_s    = 900.0;
        cfg.boost_duration_s = 40.0;
        HeaterScheduler s(cfg);
        PID pid(0.0048, 0.0036, 0.0, 0.0, 1.0);
        s.reset(350.0);
        s.setUserSetpoint(350.0);

        const int frames_to_sleep = 30000;  // 300 s at 10 ms
        const int frames_to_off   = 90000;  // 900 s at 10 ms
        FrameResult fr;
        bool slept = false, offed = false;
        for (int n = 0; n < frames_to_off + 10; ++n) {
            fr = s.stepFrame(pid, 350.0, true);
            if (!slept && fr.mode == OperatingMode::Sleep) {
                slept = true;
                checkNear(fr.active_setpoint_c, 150.0, 1e-12, "sleep drops the setpoint to 150 C");
                check(std::strcmp(HeaterScheduler::statusText(fr.status), "SLEEP") == 0,
                      "sleep status label is SLEEP");
                check(n + 1 == frames_to_sleep, "sleep entered after 300 s idle");
            }
            if (!offed && fr.mode == OperatingMode::Off) {
                offed = true;
                checkNear(fr.effective_duty, 0.0, 1e-12, "Off drives duty to 0");
                check(std::strcmp(HeaterScheduler::statusText(fr.status), "  OFF") == 0,
                      "Off status label is \"  OFF\"");
                check(n + 1 == frames_to_off, "Off entered after 900 s idle");
            }
        }
        check(slept, "idle timer entered Sleep");
        check(offed, "idle timer entered Off");

        s.notifyHandleMoved();
        fr = s.stepFrame(pid, 350.0, true);
        check(fr.mode == OperatingMode::Active, "handle movement wakes the station");
        check(fr.active_setpoint_c == 350.0, "wake restores the user setpoint");

        std::printf("-- boost --\n");
        s.toggleBoost();
        check(s.mode() == OperatingMode::Boost, "short press enters Boost");
        checkNear(s.activeSetpoint(), 400.0, 1e-12, "boost setpoint is +50 C, clamped at 400");
        fr = s.stepFrame(pid, 350.0, true);
        check(std::strcmp(HeaterScheduler::statusText(fr.status), "BOOST") == 0,
              "boost status label is BOOST");
        bool boost_ended = false;
        for (int n = 0; n < 4100; ++n) {
            fr = s.stepFrame(pid, 350.0, true);
            if (fr.mode == OperatingMode::Active) { boost_ended = true; break; }
        }
        check(boost_ended, "boost timer returns to Active after 40 s");
    }

    std::printf("-- setpoint clamps --\n");
    {
        HeaterScheduler s;
        s.setUserSetpoint(999.0);
        checkNear(s.userSetpoint(), 400.0, 1e-12, "user setpoint clamps at 400 C");
        s.setUserSetpoint(0.0);
        checkNear(s.userSetpoint(), 150.0, 1e-12, "user setpoint clamps at 150 C");
    }

    std::printf("-- unsaturated sample frame --\n");
    {
        TimingConfig cfg;
        cfg.time_to_sleep_s = 0.0;
        cfg.time_to_off_s   = 0.0;
        HeaterScheduler s(cfg);
        PID pid(0.0048, 0.0036, 0.0, 0.0, 1.0);
        s.reset(349.0);
        s.setUserSetpoint(350.0);
        const FrameResult fr = s.stepFrame(pid, 349.0, true);
        check(fr.sample_frame, "first frame after reset is a sample frame");
        check(fr.effective_duty < 0.75, "1 C below setpoint stays under the ceiling");
        checkNear(fr.post_blank_pwm_duty, fr.effective_duty / 0.75, 1e-12,
                  "post-blank duty rescales an unsaturated sample frame");
    }

    std::printf("-- derivative on measurement --\n");
    {
        PID pid(0.0048, 0.0036, 0.05, 0.0, 1.0);
        pid.setSetpoint(350.0);
        pid.update(300.0, 0.01, true);
        pid.update(310.0, 0.01, true, 0.05);
        const double d_rising = pid.dTerm();
        check(d_rising < 0.0, "rising measurement gives a negative d term");
        for (int n = 0; n < 4; ++n) pid.update(310.0, 0.01, false);
        checkNear(pid.dTerm(), d_rising, 0.0, "d term holds across non-sample frames");

        pid.update(330.0, 0.01, true, 0.05);
        check(pid.dTerm() != d_rising, "next sample refreshes the d term");

        pid.reset();
        checkNear(pid.dTerm(), 0.0, 0.0, "reset clears the d term");

        pid.setSetpoint(350.0);
        pid.update(300.0, 0.01, true);
        pid.update(310.0, 0.01, true, 0.05);
        check(pid.dTerm() != 0.0, "derivative re-established before the setpoint change");
        pid.setSetpoint(360.0);
        checkNear(pid.dTerm(), 0.0, 0.0, "setpoint change clears the d term");

        pid.setSetpoint(std::nan(""));
        checkNear(pid.setpoint(), 360.0, 0.0, "setSetpoint ignores NaN");
    }

    std::printf("-- fault and setpoint guards --\n");
    {
        TimingConfig cfg;
        cfg.time_to_sleep_s = 0.0;
        cfg.time_to_off_s   = 0.0;
        HeaterScheduler s(cfg);
        PID pid(0.0048, 0.0036, 0.0, 0.0, 1.0);
        s.reset(26.0);
        s.setUserSetpoint(320.0);

        s.setUserSetpoint(std::nan(""));
        checkNear(s.userSetpoint(), 320.0, 1e-12, "setUserSetpoint ignores NaN");

        const FrameResult fr = s.stepFrame(pid, -100.0, true);  // shorted front end
        check(fr.fault == FaultCode::SensorReadError,
              "reading below the sane floor latches SensorReadError");
        checkNear(fr.effective_duty, 0.0, 1e-12, "below-floor reading forces duty 0");

        s.setStandbyOff(false);
        check(s.mode() == OperatingMode::Fault,
              "setStandbyOff(false) leaves a latched fault in Fault");

        // A trip through the Off state must not re-arm the fault either: the
        // control block re-asserts the latch before deciding duty.
        s.setStandbyOff(true);
        s.setStandbyOff(false);
        const FrameResult held = s.stepFrameHeld(pid);
        check(held.mode == OperatingMode::Fault,
              "setStandbyOff(false) cannot escape a latched fault");
        checkNear(held.effective_duty, 0.0, 1e-12,
                  "faulted station stays off after the mode change");
    }

    std::printf("-- elapsed timers --\n");
    {
        TimingConfig cfg;
        cfg.time_to_sleep_s  = 3.0;  // 150 frames at 0.02 s
        cfg.time_to_off_s    = 0.0;
        cfg.boost_duration_s = 0.0;
        HeaterScheduler s(cfg);
        PID pid(0.0048, 0.0036, 0.0, 0.0, 1.0);
        s.reset(350.0);
        s.setUserSetpoint(350.0);

        int frames = 0;
        FrameResult fr;
        do {
            fr = s.stepFrame(pid, 350.0, true, 0.02);
            ++frames;
        } while (fr.mode != OperatingMode::Sleep && frames < 1000);
        check(fr.mode == OperatingMode::Sleep,
              "elapsed_s drives the sleep timer at a non-default cadence");
        check(frames == 150, "3 s at 0.02 s per frame enters Sleep on frame 150");

        const double temp_before = s.lastMeasuredTemp();
        const FrameResult held   = s.stepFrameHeld(pid);
        check(!held.sample_frame, "held frame consumes no measurement");
        checkNear(s.lastMeasuredTemp(), temp_before, 0.0,
                  "held frame keeps the last measurement");
        checkNear(s.displayTemp(), temp_before, 0.0, "held frame keeps the shown value");
    }

    std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "SELF-TEST OK" : "SELF-TEST FAILED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--self-test") == 0) return runSelfTests();

    const double dt    = 0.01;   // 100 Hz control frame
    const double T_end = 30.0;   // seconds
    const double SP    = 350.0;  // setpoint step target (C)

    PlantParams pp;
    ThermalPlant plant(pp);
    plant.reset(pp.Tamb);

    // Station timers off: a 30 s benchmark must not wander into Sleep.
    TimingConfig tcfg;
    tcfg.time_to_sleep_s = 0.0;
    tcfg.time_to_off_s   = 0.0;

    HeaterScheduler sched(tcfg);
    sched.reset(pp.Tamb);
    sched.setUserSetpoint(SP);

    // Gains analytically seeded for zeta = 1, wn = 1 rad/s on the linearised loop
    // (plant: tau = 1.50 s, DC gain = Pmax*Rth = 414 C per unit duty).
    PID pid(/*kp*/0.0048, /*ki*/0.0036, /*kd*/0.0, /*out_min*/0.0, /*out_max*/1.0);

    FILE* f = fopen("step_response.csv", "w");
    if (!f) { perror("step_response.csv"); return 1; }
    fprintf(f, "t_ms,setpoint,temp,duty,p,i,d\n");

    double max_sample_duty = 0.0, max_other_duty = 0.0;
    int sample_frames = 0;
    for (int n = 0; n * dt <= T_end; ++n) {
        const FrameResult fr   = sched.stepFrame(pid, plant.temp(), true);
        const double     T     = plant.step(fr.effective_duty, dt);
        if (fr.sample_frame) {
            ++sample_frames;
            if (fr.effective_duty > max_sample_duty) max_sample_duty = fr.effective_duty;
        } else if (fr.effective_duty > max_other_duty) {
            max_other_duty = fr.effective_duty;
        }
        fprintf(f, "%.0f,%.2f,%.3f,%.5f,%.4f,%.4f,%.4f\n", n * dt * 1000.0,
                fr.active_setpoint_c, T, fr.effective_duty, pid.pTerm(), pid.iTerm(),
                pid.dTerm());
    }
    fclose(f);

    const double sample_hz = 1.0 / (tcfg.frame_dt_s * tcfg.sample_stride);
    std::printf("plant: R_tip=%.1f ohm  V=%.0f  Pmax=%.1f W  tau=%.2f s\n", pp.R_tip,
                pp.Vsupply, plant.powerMax(), pp.Rth * pp.Cth);
    std::printf("timing: 100 Hz control / %.0f Hz sampling (every %dth frame), "
                "blank %.0f us of %.0f ms, d_max=%.2f (sample frame %.2f)\n",
                sample_hz, tcfg.sample_stride, tcfg.settle_us + tcfg.conv_us,
                tcfg.frame_dt_s * 1000.0, sched.cycleAverageMaxDuty(),
                sched.sampleFrameMaxDuty());
    std::printf("interlock: %d sample frames, max duty %.3f on sample frames "
                "(ceiling %.2f), %.3f elsewhere\n",
                sample_frames, max_sample_duty, sched.sampleFrameMaxDuty(), max_other_duty);
    std::printf("final temp = %.2f C   -> wrote step_response.csv\n", plant.temp());
    return 0;
}
