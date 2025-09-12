// siacci.cpp

/*
 * Notes on the Implementation

Standard Atmosphere Table: The atmosphereTable contains simplified standard atmosphere data up to 11,000 meters. For more accuracy, you can expand this table or use a more detailed model.
Siacci Coefficients: The coefficients A2, B2, and C2 are approximate values for the G1 drag function. For precise calculations, you should use empirical data or more accurate approximations.
Siacci Function: The siacciFunction is an approximation of the Siacci flat-fire function. The actual Siacci function is more complex and typically requires numerical integration or lookup tables.
Drag Calculation: The drag calculation is simplified and uses the G1 drag function. For more accurate results, consider using more detailed drag models or empirical data.
*/

#include "siacci.h"

// Constructor
Siacci::Siacci() {
    initializeAtmosphereTable();
}

// Initialize standard atmosphere table (simplified)
void Siacci::initializeAtmosphereTable() {
    // Standard Atmosphere Data (ISA) up to 11,000 meters
    atmosphereTable = {
        {0.0,    {0.0,    288.15, 101325.0, 1.225, 340.3}},
        {1000.0, {1000.0, 281.65, 89874.0, 1.112, 336.4}},
        {2000.0, {2000.0, 275.15, 79495.0, 1.007, 332.5}},
        {3000.0, {3000.0, 268.65, 70116.0, 0.909, 328.6}},
        {4000.0, {4000.0, 262.15, 61640.0, 0.819, 324.6}},
        {5000.0, {5000.0, 255.65, 54020.0, 0.736, 320.5}},
        {6000.0, {6000.0, 249.15, 47181.0, 0.660, 316.4}},
        {7000.0, {7000.0, 242.65, 41067.0, 0.590, 312.2}},
        {8000.0, {8000.0, 236.15, 35607.0, 0.526, 308.1}},
        {9000.0, {9000.0, 229.65, 30745.0, 0.467, 303.9}},
        {10000.0,{10000.0,223.15, 26436.0, 0.413, 299.5}},
        {11000.0,{11000.0,216.65, 22632.0, 0.365, 295.1}}
    };
}

// Get atmosphere data for a given altitude (linear interpolation)
AtmosphereData Siacci::getAtmosphereData(double altitude) const {
    if (altitude <= 0.0) return atmosphereTable.at(0.0);
    if (altitude >= 11000.0) return atmosphereTable.at(11000.0);

    auto it = atmosphereTable.lower_bound(altitude);
    if (it == atmosphereTable.begin()) return it->second;

    auto prevIt = std::prev(it);
    double t = (altitude - prevIt->first) / (it->first - prevIt->first);

    AtmosphereData result;
    result.altitude = altitude;
    result.temperature = prevIt->second.temperature + t * (it->second.temperature - prevIt->second.temperature);
    result.pressure = prevIt->second.pressure + t * (it->second.pressure - prevIt->second.pressure);
    result.density = prevIt->second.density + t * (it->second.density - prevIt->second.density);
    result.speedOfSound = prevIt->second.speedOfSound + t * (it->second.speedOfSound - prevIt->second.speedOfSound);

    return result;
}

// Set parameters and calculate Siacci coefficients
void Siacci::setParameters(double mass, double diameter, double dragCoeff,
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

    calculateSiacciCoefficients();
    trajectory.clear();
    trajectory.push_back({currentX, currentY, currentTime});
}

// Calculate Siacci coefficients for G1 drag function
void Siacci::calculateSiacciCoefficients() {
    double v0 = muzzleVelocity;
    double i0 = dragCoeff * 5.625e-6 * v0; // Initial retardation

    coefficients.C1 = ballisticCoefficient / 1000.0;
    coefficients.i1 = i0;
    coefficients.tau = coefficients.C1 / coefficients.i1;

    // Siacci function coefficients (approximate for G1 drag function)
    coefficients.A2 = 1.0;
    coefficients.B2 = 1.4;
    coefficients.C2 = 3.4;
}

// Siacci function approximation
double Siacci::siacciFunction(double argument) const {
    // Approximation of the Siacci function for G1 drag function
    return coefficients.A2 * log(1.0 + coefficients.B2 * argument) -
           coefficients.C2 * argument;
}

// Calculate drag based on velocity and altitude
double Siacci::calculateDrag(double velocity, double altitude) const {
    AtmosphereData atm = getAtmosphereData(altitude);
    double mach = velocity / atm.speedOfSound;
    // Simplified drag calculation using G1 drag function
    double dragCoefficient = dragCoeff * (1.0 + 0.2 * mach * mach);
    return 0.5 * atm.density * velocity * velocity * M_PI * diameter * diameter / 4 * dragCoefficient;
}

// Step through the trajectory
void Siacci::step(double dt) {
    // Siacci's method uses analytical solutions for flat-fire trajectories
    // This is a simplified numerical integration for demonstration

    double v = muzzleVelocity * exp(-currentTime / coefficients.tau);
    double theta = launchAngle - siacciFunction(currentTime / coefficients.tau) / coefficients.C1;

    double drag = calculateDrag(v, currentY);
    double dragAccel = drag / mass;

    currentX += v * cos(theta) * dt;
    currentY += v * sin(theta) * dt - 0.5 * 9.81 * dt * dt;
    currentTime += dt;

    trajectory.push_back({currentX, currentY, currentTime});
}

// Get the trajectory
const std::vector<std::array<double, 3>>& Siacci::getTrajectory() const {
    return trajectory;
}
