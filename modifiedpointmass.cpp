// modifiedpointmass.cpp
#include "modifiedpointmass.h"
#include <cmath>

ModifiedPointMass::ModifiedPointMass() {}

void ModifiedPointMass::setParameters(double mass, double diameter, double dragCoeff,
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
    state = {0, 0, muzzleVelocity, launchAngle, 0};
    trajectory.clear();
    trajectory.push_back({state[0], state[1], state[4]});
}

void ModifiedPointMass::step(double dt) {
    double v = state[2];
    double dragForce = 0.5 * 1.225 * v * v * M_PI * diameter * diameter / 4 * dragCoeff;
    double dragAccel = dragForce / mass;

    state[2] += (-dragAccel - 9.81 * sin(state[3])) * dt;
    state[1] += v * sin(state[3]) * dt;
    state[0] += v * cos(state[3]) * dt;
    state[3] += (-9.81 * cos(state[3]) / v) * dt;
    state[4] += dt;

    trajectory.push_back({state[0], state[1], state[4]});
}

const std::vector<std::array<double, 3>>& ModifiedPointMass::getTrajectory() const {
    return trajectory;
}
