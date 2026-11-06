// Host simulation harness: runs the SAME control code (lib/control) that will run
// on the ESP32, against the thermal plant, and writes a step-response CSV.
//
//   output: step_response.csv   (t_ms,setpoint,temp,duty,p,i,d)
//   build : pio run            (env:native)
//   run   : .pio/build/native/program
#include <cstdio>
#include "pid.h"
#include "plant.h"

int main() {
    const double dt     = 0.01;    // 100 Hz control rate
    const double T_end  = 30.0;    // seconds
    const double SP     = 350.0;   // setpoint step target (C)

    PlantParams pp;
    ThermalPlant plant(pp);
    plant.reset(pp.Tamb);

    // Gains analytically seeded for zeta = 1, wn = 1 rad/s on the linearised loop
    // (plant: tau = 1.50 s, DC gain = Pmax*Rth = 414 C per unit duty).
    PID pid(/*kp*/0.0048, /*ki*/0.0036, /*kd*/0.0, /*out_min*/0.0, /*out_max*/1.0);
    pid.setSetpoint(SP);

    FILE* f = fopen("step_response.csv", "w");
    if (!f) { perror("step_response.csv"); return 1; }
    fprintf(f, "t_ms,setpoint,temp,duty,p,i,d\n");

    for (int n = 0; n * dt <= T_end; ++n) {
        const double duty = pid.update(plant.temp(), dt);
        const double T    = plant.step(duty, dt);
        fprintf(f, "%.0f,%.2f,%.3f,%.5f,%.4f,%.4f,%.4f\n",
                n * dt * 1000.0, pid.setpoint(), T, duty,
                pid.pTerm(), pid.iTerm(), pid.dTerm());
    }
    fclose(f);

    printf("plant: R_tip=%.1f ohm  V=%.0f  Pmax=%.1f W  tau=%.2f s\n",
           pp.R_tip, pp.Vsupply, plant.powerMax(), pp.Rth * pp.Cth);
    printf("final temp = %.2f C   -> wrote step_response.csv\n", plant.temp());
    return 0;
}
