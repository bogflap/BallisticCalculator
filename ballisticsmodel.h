// ballisticsmodel.h
#pragma once
#include <vector>
#include <array>

class BallisticsModel {
public:
    virtual ~BallisticsModel() = default;
    virtual void step(double dt) = 0;
    virtual const std::vector<std::array<double, 3>>& getTrajectory() const = 0;
    virtual void setParameters(double mass, double diameter, double dragCoeff,
                               double muzzleVelocity, double launchAngle,
                               double windSpeed, double windDirection,
                               double latitude, double scopeHeight) = 0;
};
