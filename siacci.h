// siacci.h
#pragma once
#include "ballisticsmodel.h"
#include <vector>
#include <array>
#include <cmath>
#include <map>

// Standard Atmosphere Data (simplified)
struct AtmosphereData {
    double altitude; // in meters
    double temperature; // in Kelvin
    double pressure; // in Pascals
    double density; // in kg/m³
    double speedOfSound; // in m/s
};

// Siacci Coefficients for G1 Drag Function
struct SiacciCoefficients {
    double C1; // Ballistic coefficient
    double i1; // Function of muzzle velocity
    double A2, B2, C2; // Coefficients for the Siacci function
    double tau; // Time constant
};

// Main Siacci Model Class
class Siacci : public BallisticsModel {
public:
    Siacci();
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
    SiacciCoefficients coefficients;
    std::map<double, AtmosphereData> atmosphereTable;

    void initializeAtmosphereTable();
    AtmosphereData getAtmosphereData(double altitude) const;
    void calculateSiacciCoefficients();
    double siacciFunction(double argument) const;
    double calculateDrag(double velocity, double altitude) const;
};
