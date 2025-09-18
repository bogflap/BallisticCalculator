// dof6.cpp
#include "dof6.h"
#include <cmath>

DOF6::DOF6() {}

// dof6.cpp
void DOF6::setParameters(double mass, double diameter, double dragCoeff,
                         double muzzleVelocity, double launchAngle,
                         double windSpeed, double windDirection,
                         double latitude, double scopeHeight) {
    this->mass = mass;
    this->diameter = diameter;
    this->dragCoeff = dragCoeff;
    this->muzzleVelocity = muzzleVelocity;
    this->launchAngle = launchAngle;
    this->windSpeed = windSpeed;
    this->windDirection = windDirection;
    this->latitude = latitude;
    this->scopeHeight = scopeHeight;

    // Initialize the state vector with scope height adjustment
    state = {0, -scopeHeight, muzzleVelocity * cos(launchAngle),
             muzzleVelocity * sin(launchAngle), 0, -9.81};

    trajectory.clear();
    trajectory.push_back({state[0], state[1], 0});
}

void DOF6::step(double dt) {
    double v = sqrt(state[2] * state[2] + state[3] * state[3]);
    double dragForce = 0.5 * 1.225 * v * v * M_PI * diameter * diameter / 4 * dragCoeff;
    double dragAccel = dragForce / mass;

    double windX = windSpeed * cos(windDirection);
    double windY = windSpeed * sin(windDirection);

    state[2] += (-dragAccel * state[2] / v + windX) * dt;
    state[3] += (-9.81 - dragAccel * state[3] / v + windY) * dt;
    state[0] += state[2] * dt;
    state[1] += state[3] * dt;
    state[4] += dt;

    trajectory.push_back({state[0], state[1], state[4]});
}

const std::vector<std::array<double, 3>>& DOF6::getTrajectory() const {
    return trajectory;
}
