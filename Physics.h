#pragma once
#include <string>
#include "Parameters.h" // Needed for member variable 'p'
#include "MathFunctions.h"


class Forces
{
public:
    Forces();
    Forces(std::string axis, PlaneParameters p);

    void calcAerodynamicForce(PlaneParameters& p);
    void calcThrust(PlaneParameters& p, double dt);
    double summation_of_Forces(PlaneParameters& p);

    double air_density;
    double aerodynamicForceResultant;
    double actual_Thrust;

private:
    void calcAirDensity(double altitude);

    double mass;
    std::string axis; // "x", "y", or "z"

    xyz velocity_bodyFrame;
    xyz position_earthFrame;
    
    // Aerodynamic constants
    const double CL0 = 0.0;
    const double CL_alpha = 0.0;
    const double CD0 = 0.0;
    const double CD_alpha = 0.0;
    const double wingArea = 0.0;
    const double CY_beta = 0.0;
    const double wingSettingAngle = 0.0;
    const double k = 0.0;
    const double tailArea = 0.0;
    const double CL_alpha_tailWing = 0.0;
    const double CL0_tailWing = 0.0;
    const double CD0_tailWing = 0.0;
    const double k_tailWing = 0.0;
    

    // Thrust variables
    const double maxThrust_at_sea_level;
    const double air_density_sea_level;
    double throttle;
    double fuelPercentage;
};

// The Master Orchestrator
void updatePhysics(PlaneParameters& p, double dt, double pilotThrottle, double elevator, double aileron, double rudder);

// Atmosphere
double getDynamicPressure(const PlaneParameters& p);

// Calculate Forces & Torques
xyz getLinearBodyForces(PlaneParameters& p, double dt, double throttle);
xyz getRotationalTorques(const PlaneParameters& p, double Q, double elevator, double aileron, double rudder);

// Integrate Dynamics
void updateRotationalDynamics(PlaneParameters& p, xyz torques, double dt);
void updateLinearDynamics(PlaneParameters& p, xyz body_forces, double dt);

// Utility
double calcGravitationalAccelerationCONST(double latitude_rad, double altitude);
