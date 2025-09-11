// ballistics.h
#pragma once
#include <vector>
#include <cmath>

struct State {
    double x, y, z;       // Position (m)
    double vx, vy, vz;     // Velocity (m/s)
    double roll, pitch, yaw; // Orientation (rad)
    double time;          // Time (s)
};

class Ballistics6DOF {
public:
    Ballistics6DOF(double mass, double diameter, double dragCoeff, double muzzleVelocity, double launchAngle,
                   double windSpeed = 0, double windDirection = 0, double latitude = 0);
    void step(double dt);
    const std::vector<State>& getTrajectory() const { return trajectory; }
    void clearTrajectory() { trajectory.clear(); }

private:
    double mass, diameter, dragCoeff, muzzleVelocity, launchAngle;
    double windSpeed, windDirection, latitude;
    std::vector<State> trajectory;
    State currentState;
};
