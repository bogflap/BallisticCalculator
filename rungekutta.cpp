// rungekutta.cpp
#include "rungekutta.h"
#include <cmath>

RungeKutta::RungeKutta() {}

void RungeKutta::setParameters(double mass, double diameter, double dragCoeff,
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
    state = {0, 0, muzzleVelocity * cos(launchAngle), muzzleVelocity * sin(launchAngle), 0};
    trajectory.clear();
    trajectory.push_back({state[0], state[1], state[4]});
}

void RungeKutta::step(double dt) {
    auto dxdt = [this](const std::array<double, 5> &s) {
        double v = sqrt(s[2] * s[2] + s[3] * s[3]);
        double dragForce = 0.5 * 1.225 * v * v * M_PI * diameter * diameter / 4 * dragCoeff;
        double dragAccel = dragForce / mass;
        return std::array<double, 5>{
            s[2],
            s[3],
            -dragAccel * s[2] / v,
            -9.81 - dragAccel * s[3] / v,
            1.0
        };
    };

    std::array<double, 5> k1 = dxdt(state);
    std::array<double, 5> k2 = dxdt({state[0] + dt/2 * k1[0], state[1] + dt/2 * k1[1], state[2] + dt/2 * k1[2], state[3] + dt/2 * k1[3], state[4] + dt/2 * k1[4]});
    std::array<double, 5> k3 = dxdt({state[0] + dt/2 * k2[0], state[1] + dt/2 * k2[1], state[2] + dt/2 * k2[2], state[3] + dt/2 * k2[3], state[4] + dt/2 * k2[4]});
    std::array<double, 5> k4 = dxdt({state[0] + dt * k3[0], state[1] + dt * k3[1], state[2] + dt * k3[2], state[3] + dt * k3[3], state[4] + dt * k3[4]});

    state[0] += dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
    state[1] += dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
    state[2] += dt / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
    state[3] += dt / 6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
    state[4] += dt / 6 * (k1[4] + 2 * k2[4] + 2 * k3[4] + k4[4]);

    trajectory.push_back({state[0], state[1], state[4]});
}

const std::vector<std::array<double, 3>>& RungeKutta::getTrajectory() const {
    return trajectory;
}
