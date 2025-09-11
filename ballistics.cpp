// ballistics.cpp
#include "ballistics.h"
#include <cmath>

Ballistics6DOF::Ballistics6DOF(double mass, double diameter, double dragCoeff, double muzzleVelocity, double launchAngle,
                               double windSpeed, double windDirection, double latitude)
    : mass(mass), diameter(diameter), dragCoeff(dragCoeff), muzzleVelocity(muzzleVelocity), launchAngle(launchAngle),
    windSpeed(windSpeed), windDirection(windDirection), latitude(latitude) {
    currentState = {0, 0, 0, muzzleVelocity * cos(launchAngle), muzzleVelocity * sin(launchAngle), 0, 0, launchAngle, 0, 0};
    trajectory.push_back(currentState);
}

void Ballistics6DOF::step(double dt) {
    // Velocity magnitude
    double v = sqrt(currentState.vx*currentState.vx + currentState.vy*currentState.vy + currentState.vz*currentState.vz);

    // Drag force
    double dragForce = 0.5 * 1.225 * v * v * M_PI * diameter * diameter / 4 * dragCoeff;
    double dragAccel = dragForce / mass;

    // Wind effect (simplified)
    double windX = windSpeed * cos(windDirection);
    double windY = windSpeed * sin(windDirection);

    // Coriolis effect (simplified)
    double coriolisX = 2 * v * 7.2921e-5 * sin(latitude) * currentState.vz;
    double coriolisY = -2 * v * 7.2921e-5 * sin(latitude) * currentState.vx;

    // Update velocity and position
    currentState.vx += (-dragAccel * currentState.vx / v + windX + coriolisX) * dt;
    currentState.vy += (-9.81 - dragAccel * currentState.vy / v + windY + coriolisY) * dt;
    currentState.x += currentState.vx * dt;
    currentState.y += currentState.vy * dt;
    currentState.time += dt;

    trajectory.push_back(currentState);
}
