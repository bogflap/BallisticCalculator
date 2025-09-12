// pejsa.cpp
#include "pejsa.h"

Pejsa::Pejsa() {}

void Pejsa::setParameters(double mass, double diameter, double dragCoeff,
                          double muzzleVelocity, double launchAngle,
                          double windSpeed, double windDirection, double latitude) {
    this->mass = mass;
    this->diameter = diameter;
    this->dragCoeff = dragCoeff;
    this->muzzleVelocity = muzzleVelocity;
    this->launchAngle = launchAngle;
    this->windSpeed = windSpeed;
    this->windDirection = windDirection;
    this->latitude = latitude;
    this->currentTime = 0.0;
    this->currentX = 0.0;
    this->currentY = 0.0;
    this->ballisticCoefficient = mass / (diameter * diameter * dragCoeff);
    trajectory.clear();
    trajectory.push_back({currentX, currentY, currentTime});
}

void Pejsa::step(double dt) {
    // Pejsa's model is an empirical approximation
    // This is a simplified implementation for demonstration purposes
    double v = muzzleVelocity * exp(-dragCoeff * currentTime / ballisticCoefficient);
    double theta = launchAngle - (9.81 * currentTime) / muzzleVelocity;

    currentX += v * cos(theta) * dt;
    currentY += v * sin(theta) * dt - 0.5 * 9.81 * dt * dt;
    currentTime += dt;

    trajectory.push_back({currentX, currentY, currentTime});
}

const std::vector<std::array<double, 3>>& Pejsa::getTrajectory() const {
    return trajectory;
}
