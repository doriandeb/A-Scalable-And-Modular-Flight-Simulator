#pragma once
#include <string>
#include <vector>
#include "MathFunctions.h"

enum EngineType { PISTON, TURBOPROP, JET }; // Rocket

using namespace std;

class PlaneParameters {
public:
    PlaneParameters(); // Constructor declaration

    // **** General Aircraft Info ****
    const string name = "Boeing 747-100";
    const string type = "Commercial Airliner";

    // **** Mass and Inertia ****
    const double max_fuel = 160000.0;         // [kg] Maximum fuel capacity
    const double mass_planeOnly = 288000.0;   // [kg] Empty weight (Zero Fuel Weight)

    double current_fuel_mass = 160000.0;      // [kg] Current fuel onboard (decreases during flight)
    double fuelPercentage = 0.0;              // [%] Ratio of current_fuel_mass to max_fuel
    double mass = 0.0;                        // [kg] Total current mass (plane + fuel)

    // Moments of inertia [kg*m^2] (Kept mutable in case engine updates them as fuel burns)
    double Ixx = 24700000.0;
    double Iyy = 44900000.0;
    double Izz = 67300000.0;
    double Ixz = 1315000.0;

    // Thrust Specific Fuel Consumption
    const double TSFC = 0.000015;             // [kg/(N*s)] Rate of fuel burn per unit of thrust

    // **** Geometry & Aerodynamics (Airframe Constants) ****

    const double wingArea = 511.0;            // [m^2] Reference area for main wings
    const double wingspan = 59.6;             // [m] Tip-to-tip span
    const double wingSettingAngle = 0.0;      // [rad] Angle of incidence of the main wing

    const double k = 0.05;                    // [-] Induced drag factor
    const double CL0 = 0.25;//0.25                  // [-] Base lift coefficient at zero AoA
    const double CL_alpha = 4.5;              // [1/rad] Lift curve slope
    const double CD0 = 0.015;                 // [-] Zero-lift drag coefficient (parasitic drag)
    const double CD_alpha = 0.05;             // [-] Induced drag multiplier
    const double CY_beta = -0.01;             // [1/rad] Side force derivative (directional stability)

    const double tailArea = 130.0;            // [m^2] Reference area for horizontal stabilizer
    const double CL_alpha_tailWing = 4.5;     // [1/rad] Lift curve slope for tail
    const double CL0_tailWing = 0.015;        // [-] Base lift coefficient for tail
    const double CD0_tailWing = 0.008;        // [-] Parasitic drag for tail
    const double k_tailWing = 0.045;          // [-] Induced drag factor for tail

    const double S = 511.0;
    const double b = 59.6;
    const double c = S / b;  // 8.574 m

    const double CL_delta_aileron = -0.0461;
    const double CL_beta = -0.221;
    const double CL_p = -0.45;

    const double CM_delta_elevator = -1.34;
    const double CM_alpha = -1.26;
    const double CM_q = -20.8;

    const double CN_delta_rudder = -0.109;
    const double CN_beta = 0.15;
    const double CN_r = -0.3;

    // **** Engine & Thrust ****
    const EngineType engineType = JET;
    const double maxThrust_at_sea_level = 800000.0; // [N] Max combined thrust (4 engines x ~200kN)
    const double propEfficiency = 0.8;        // [-] Propeller efficiency (unused for jets)
    const double maxPower = 0.0;              // [W] Max power (used for piston/turboprop)

    double throttle = 1.0;                    // [-] Pilot input ratio (0.0 to 1.0)
    double actual_Thrust = 0.0;               // [N] Current thrust output (lags behind throttle input)

    // **** Environment ****
    double air_density = 1.225;               // [kg/m^3] Local atmospheric density (changes with altitude)
    const double air_density_sea_level = 1.225; // [kg/m^3] Standard ISA sea-level density

    // **** State Variables ****
    // Earth-Centered, Earth-Fixed (ECEF) Frame
    xyz position_earthFrame = { 0.0, 0.0, 0.0 }; // [m] Cartesian coordinates from Earth center
    xyz velocity_earthFrame = { 0.0, 0.0, 0.0 }; // [m/s] Linear velocity relative to Earth

    // Body Frame (X = Forward, Y = Right, Z = Down)
    xyz velocity_bodyFrame = { 0.0, 0.0, 0.0 };  // [m/s] Local velocities (u, v, w)
    xyz angles = { 0.0, 0.0, 0.0 };              // [rad] Euler angles (Roll, Pitch, Yaw)
    xyz omega = { 0.0, 0.0, 0.0 };               // [rad/s] Angular body rates (p, q, r)

    // Heading
    double heading = 0.0;                     // [deg] Compass heading (0 = North, 90 = East)
};