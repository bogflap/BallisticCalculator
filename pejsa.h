// pejsa.h
#pragma once
#include "ballisticsmodel.h"
#include <vector>
#include <array>
#include <cmath>

class Pejsa : public BallisticsModel {
public:
    Pejsa();
    void step(double dt) override;
    const std::vector<std::array<double, 3>>& getTrajectory() const override;
    void setParameters(double mass, double diameter, double dragCoeff,
                       double muzzleVelocity, double launchAngle,
                       double windSpeed, double windDirection,
                       double latitude, double scopeHeight) override;

private:
    std::vector<std::array<double, 3>> trajectory;
    double mass, diameter, dragCoeff, muzzleVelocity, launchAngle;
    double windSpeed, windDirection, latitude, scopeHeight;
    double currentTime, currentX, currentY;
    double ballisticCoefficient;
};
