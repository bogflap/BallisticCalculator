/**
 * @file rungekutta.cpp
 * @brief Implementation of the Runge-Kutta ballistics model.
 *
 * This file contains the implementation of the Runge-Kutta method for
 * calculating bullet trajectories. The Runge-Kutta method is a numerical
 * technique for solving ordinary differential equations, providing better
 * accuracy than simpler methods like Euler's method.
 */

#include "rungekutta.h"
#include <cmath>

/**
 * @brief Constructor for the RungeKutta class.
 *
 * Initializes a new instance of the RungeKutta ballistics model.
 * The constructor doesn't need to do anything special since all
 * initialization happens in the setParameters method.
 */
RungeKutta::RungeKutta() {
    // Initialize scope height to a default value
    scopeHeight = 0.0;
}

/**
 * @brief Sets the parameters for the ballistics model.
 *
 * Initializes the model with the physical parameters of the bullet and
 * environmental conditions. This method should be called before running
 * any trajectory calculations.
 *
 * @param mass The mass of the bullet in kilograms.
 * @param diameter The diameter of the bullet in meters.
 * @param dragCoeff The drag coefficient of the bullet.
 * @param muzzleVelocity The initial velocity of the bullet in meters per second.
 * @param launchAngle The launch angle in radians.
 * @param windSpeed The wind speed in meters per second.
 * @param windDirection The wind direction in radians.
 * @param latitude The latitude for Coriolis effect calculations (not used in this model).
 * @param scopeHeight The height of the scope centre line abobe the barrel center line in meters
 */
void RungeKutta::setParameters(double mass, double diameter, double dragCoeff,
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
    // The bullet starts at y = -scopeHeight because the scope is above the barrel
    state = {
        0.0,  // Initial x position
        -scopeHeight,  // Initial y position (below scope line)
        muzzleVelocity * cos(launchAngle),  // Initial x velocity
        muzzleVelocity * sin(launchAngle),  // Initial y velocity
        0.0   // Initial time
    };

    // Clear any existing trajectory data
    trajectory.clear();

    // Add the initial point to the trajectory
    trajectory.push_back({state[0], state[1], state[4]});
}

/**
 * @brief Calculates the drag force on the bullet.
 *
 * Computes the drag force based on the bullet's velocity, using the
 * standard drag equation: F_d = 0.5 * ρ * v² * C_d * A
 *
 * @param velocity The current velocity of the bullet in meters per second.
 * @return The drag force in Newtons.
 */
double RungeKutta::calculateDrag(double velocity) const {
    // Air density at sea level (kg/m³)
    const double airDensity = 1.225;

    // Cross-sectional area of the bullet (m²)
    double area = M_PI * diameter * diameter / 4.0;

    // Drag force: F_d = 0.5 * ρ * v² * C_d * A
    // Where:
    //   ρ = air density
    //   v = velocity
    //   C_d = drag coefficient
    //   A = cross-sectional area
    return 0.5 * airDensity * velocity * velocity * dragCoeff * area;
}

/**
 * @brief Defines the system of differential equations for the bullet's motion.
 *
 * This function represents the system of ODEs that describe the bullet's motion,
 * including altitude-dependent air density and distance-dependent wind effects.
 *
 * @param s The current state vector (x, y, vx, vy, t).
 * @return The derivatives of the state vector (dx/dt, dy/dt, dvx/dt, dvy/dt, dt/dt).
 */
std::array<double, 5> RungeKutta::getDerivatives(const std::array<double, 5> &s) const {
    double x = s[0];  // Horizontal position (m)
    double y = s[1];  // Vertical position/altitude (m)
    double vx = s[2];
    double vy = s[3];

    // Calculate velocity magnitude
    double v = sqrt(vx * vx + vy * vy);

    // Calculate air density based on altitude (y)
    // Using the barometric formula: ρ = ρ₀ * e^(-y/H)
    const double seaLevelDensity = 1.225;  // kg/m³
    const double scaleHeight = 8500.0;    // meters

    // Ensure y is not negative (below sea level) for density calculation
    double altitude = std::max(y, 0.0);
    double airDensity = seaLevelDensity * exp(-altitude / scaleHeight);

    // Calculate cross-sectional area of the bullet
    double area = M_PI * diameter * diameter / 4.0;

    // Calculate drag force using altitude-dependent air density
    double dragForce = 0.5 * airDensity * v * v * dragCoeff * area;

    // Calculate drag acceleration in x and y directions
    double dragAx = -(dragForce / mass) * (vx / v);
    double dragAy = -(dragForce / mass) * (vy / v);

    // Wind components with distance-dependent variation
    // Model wind as potentially changing with distance (x)
    // Using a simple model where wind speed can vary linearly with distance
    const double windVariationFactor = 0.0001;  // Small factor for wind variation with distance
    double distanceDependentWindSpeed = windSpeed * (1.0 + windVariationFactor * x);

    // Calculate wind components using the distance-dependent wind speed
    double windX = distanceDependentWindSpeed * cos(windDirection);
    double windY = distanceDependentWindSpeed * sin(windDirection);

    // Add a small vertical wind component that varies with altitude
    // This simulates thermal effects or altitude-dependent wind patterns
    const double thermalEffectFactor = 0.00005;  // Small factor for altitude-dependent vertical wind
    windY += thermalEffectFactor * altitude;

    // Define the derivatives:
    // dx/dt = vx (horizontal velocity)
    // dy/dt = vy (vertical velocity)
    // dvx/dt = dragAx + windX (horizontal acceleration)
    // dvy/dt = -g + dragAy + windY (vertical acceleration, including gravity)
    // dt/dt = 1 (time always increases at 1 second per second)
    return std::array<double, 5>{
        vx,                          // dx/dt
        vy,                          // dy/dt
        dragAx + windX,             // dvx/dt
        -9.81 + dragAy + windY,     // dvy/dt (including gravity)
        1.0                          // dt/dt
    };
}

/**
 * @brief Advances the simulation by one time step using the Runge-Kutta method.
 *
 * The Runge-Kutta method is a fourth-order numerical integration technique
 * that provides better accuracy than simpler methods like Euler's method.
 * It works by calculating four intermediate slopes (k1, k2, k3, k4) and
 * using a weighted average of these slopes to advance the state.
 *
 * @param dt The time step size in seconds.
 */
void RungeKutta::step(double dt) {
    // Get the current state
    std::array<double, 5> currentState = state;

    // Calculate the four intermediate slopes (k1, k2, k3, k4)

    // k1 is the slope at the beginning of the interval
    std::array<double, 5> k1 = getDerivatives(currentState);

    // k2 is the slope at the midpoint of the interval, using k1 to estimate the state
    std::array<double, 5> tempState2 = {
        currentState[0] + dt/2 * k1[0],
        currentState[1] + dt/2 * k1[1],
        currentState[2] + dt/2 * k1[2],
        currentState[3] + dt/2 * k1[3],
        currentState[4] + dt/2 * k1[4]
    };
    std::array<double, 5> k2 = getDerivatives(tempState2);

    // k3 is another slope at the midpoint, using k2 to estimate the state
    std::array<double, 5> tempState3 = {
        currentState[0] + dt/2 * k2[0],
        currentState[1] + dt/2 * k2[1],
        currentState[2] + dt/2 * k2[2],
        currentState[3] + dt/2 * k2[3],
        currentState[4] + dt/2 * k2[4]
    };
    std::array<double, 5> k3 = getDerivatives(tempState3);

    // k4 is the slope at the end of the interval, using k3 to estimate the state
    std::array<double, 5> tempState4 = {
        currentState[0] + dt * k3[0],
        currentState[1] + dt * k3[1],
        currentState[2] + dt * k3[2],
        currentState[3] + dt * k3[3],
        currentState[4] + dt * k3[4]
    };
    std::array<double, 5> k4 = getDerivatives(tempState4);

    // Update the state using a weighted average of the four slopes
    // The weights are chosen to give a fourth-order accurate result
    state[0] += dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);  // Update x position
    state[1] += dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);  // Update y position
    state[2] += dt / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);  // Update x velocity
    state[3] += dt / 6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);  // Update y velocity
    state[4] += dt / 6 * (k1[4] + 2 * k2[4] + 2 * k3[4] + k4[4]);  // Update time

    // Add the current state to the trajectory
    trajectory.push_back({state[0], state[1], state[4]});
}

/**
 * @brief Returns the calculated trajectory.
 *
 * @return A vector of arrays, where each array contains x, y, and time.
 */
const std::vector<std::array<double, 3>>& RungeKutta::getTrajectory() const {
    return trajectory;
}
