#include "plant.h"

ThermalPlant::ThermalPlant(const PlantParams& p) : p_(p), T_(p.Tamb) {}

void ThermalPlant::reset(double T0) { T_ = T0; }

double ThermalPlant::step(double duty, double dt) {
    const double P    = duty * powerMax();          // W delivered to the node
    const double loss = (T_ - p_.Tamb) / p_.Rth;    // W lost to ambient
    T_ += (P - loss) / p_.Cth * dt;                 // forward Euler (dt << tau)
    return T_;
}
