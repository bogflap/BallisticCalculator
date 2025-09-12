// modifiedeuler.cpp
#include "modifiedeuler.h"

ModifiedEuler::ModifiedEuler() {
    // Standard air density at sea level (kg/m³)
    airDensity = 1.225;
}

void ModifiedEuler::setParameters(double mass, double diameter, double dragCoeff,
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

    // Initial state: x, y, vx, vy, time
    state = {0.0, 0.0, muzzleVelocity * cos(launchAngle), muzzleVelocity * sin(launchAngle), 0.0};
    trajectory.clear();
    trajectory.push_back({state[0], state[1], state[4]});
}

double ModifiedEuler::calculateDrag(double velocity) const {
    // Drag force: F_d = 0.5 * ρ * v² * C_d * A
    // A = π * (diameter/2)²
    double area = M_PI * diameter * diameter / 4.0;
    return 0.5 * airDensity * velocity * velocity * dragCoeff * area;
}

void ModifiedEuler::step(double dt) {
    // Current state
    double x = state[0];
    double y = state[1];
    double vx = state[2];
    double vy = state[3];
    double t = state[4];

    // Current velocity magnitude
    double v = sqrt(vx * vx + vy * vy);

    // Drag acceleration
    double drag = calculateDrag(v);
    double dragAx = - (drag / mass) * (vx / v);
    double dragAy = - (drag / mass) * (vy / v);

    // Wind effect
    double windX = windSpeed * cos(windDirection);
    double windY = windSpeed * sin(windDirection);

    // Predictor step (Euler)
    double xPred = x + vx * dt;
    double yPred = y + vy * dt;
    double vxPred = vx + (dragAx + windX) * dt;
    double vyPred = vy + (-9.81 + dragAy + windY) * dt;

    // Velocity magnitude at predictor step
    double vPred = sqrt(vxPred * vxPred + vyPred * vyPred);

    // Corrector step (using predictor values)
    double dragPred = calculateDrag(vPred);
    double dragAxPred = - (dragPred / mass) * (vxPred / vPred);
    double dragAyPred = - (dragPred / mass) * (vyPred / vPred);

    // Average accelerations
    double avgAx = 0.5 * (dragAx + dragAxPred + windX);
    double avgAy = 0.5 * (-9.81 + dragAy + -9.81 + dragAyPred + windY);

    // Update state using average accelerations
    state[0] = x + vx * dt;
    state[1] = y + vy * dt;
    state[2] = vx + avgAx * dt;
    state[3] = vy + avgAy * dt;
    state[4] = t + dt;

    trajectory.push_back({state[0], state[1], state[4]});
}

const std::vector<std::array<double, 3>>& ModifiedEuler::getTrajectory() const {
    return trajectory;
}
