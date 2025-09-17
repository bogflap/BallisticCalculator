// rungekutta.h
#pragma once
#include "ballisticsmodel.h"
#include <vector>
#include <array>

class RungeKutta : public BallisticsModel {
public:
    RungeKutta();
    void step(double dt) override;
    const std::vector<std::array<double, 3>>& getTrajectory() const override;
    void setParameters(double mass, double diameter, double dragCoeff,
                       double muzzleVelocity, double launchAngle,
                       double windSpeed, double windDirection, double latitude) override;
    std::array<double, 5> getDerivatives(const std::array<double, 5> &s) const;
    double calculateDrag(double velocity) const;

private:
    std::vector<std::array<double, 3>> trajectory;
    double mass, diameter, dragCoeff, muzzleVelocity, launchAngle;
    double windSpeed, windDirection, latitude;
    std::array<double, 5> state; // {x, y, vx, vy, time}
};
