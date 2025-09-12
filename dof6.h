// dof6.h
#pragma once
#include "ballisticsmodel.h"
#include <vector>
#include <array>

class DOF6 : public BallisticsModel {
public:
    DOF6();
    void step(double dt) override;
    const std::vector<std::array<double, 3>>& getTrajectory() const override;
    void setParameters(double mass, double diameter, double dragCoeff,
                       double muzzleVelocity, double launchAngle,
                       double windSpeed, double windDirection, double latitude) override;

private:
    std::vector<std::array<double, 3>> trajectory; // {x, y, time}
    double mass, diameter, dragCoeff, muzzleVelocity, launchAngle;
    double windSpeed, windDirection, latitude;
    std::array<double, 6> state; // {x, y, vx, vy, time}
};
