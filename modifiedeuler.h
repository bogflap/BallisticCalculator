// modifiedeuler.h
#pragma once
#include "ballisticsmodel.h"
#include <vector>
#include <array>
#include <cmath>

class ModifiedEuler : public BallisticsModel {
public:
    ModifiedEuler();
    void step(double dt) override;
    const std::vector<std::array<double, 3>>& getTrajectory() const override;
    void setParameters(double mass, double diameter, double dragCoeff,
                       double muzzleVelocity, double launchAngle,
                       double windSpeed, double windDirection, double latitude) override;

private:
    std::vector<std::array<double, 3>> trajectory;
    double mass, diameter, dragCoeff, muzzleVelocity, launchAngle;
    double windSpeed, windDirection, latitude;
    std::array<double, 5> state; // {x, y, vx, vy, time}
    double airDensity;
    double calculateDrag(double velocity) const;
};
