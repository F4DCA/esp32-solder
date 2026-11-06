#pragma once
// First-order thermal model of the T12 tip's *sensed* node (the heating-element
// film the thermocouple sits on, see the two-node discussion in INFO.md).
//
//   Cth * dT/dt = P_in - (T - Tamb)/Rth ,   P_in = duty * Vsupply^2 / R_tip
//
// Seeded from: R_tip = 8.5 ohm, V = 24 V (Pmax ~= 68 W) and "26 -> 350 C in ~2 s".
struct PlantParams {
    double Tamb    = 26.0;   // C    ambient
    double Rth     = 6.11;   // K/W  node -> ambient thermal resistance
    double Cth     = 0.245;  // J/K  node thermal capacitance  (tau = Rth*Cth = 1.50 s)
    double R_tip   = 8.5;    // ohm
    double Vsupply = 24.0;   // V
};

class ThermalPlant {
public:
    explicit ThermalPlant(const PlantParams& p = PlantParams{});
    void   reset(double T0);
    double step(double duty, double dt);        // duty 0..1, dt seconds -> returns T (C)
    double temp() const { return T_; }
    double powerMax() const { return p_.Vsupply * p_.Vsupply / p_.R_tip; }  // W
private:
    PlantParams p_;
    double T_;
};
