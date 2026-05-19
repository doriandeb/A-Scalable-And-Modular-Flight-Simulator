
#include "Physics.h"
#include "MathFunctions.h"
#include "PrimaryFlightDisplay.h"
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;


Forces::Forces(string axisName, PlaneParameters p)
    : actual_Thrust(p.actual_Thrust),
    fuelPercentage(p.fuelPercentage),
    mass(p.mass),
    axis(axisName),
    air_density(p.air_density),
    air_density_sea_level(p.air_density_sea_level),
    maxThrust_at_sea_level(p.maxThrust_at_sea_level),
    throttle(p.throttle),
    CL0(p.CL0),
    CL_alpha(p.CL_alpha),
    CD0(p.CD0),
    CD_alpha(p.CD_alpha),
    wingArea(p.wingArea),
    wingSettingAngle(p.wingSettingAngle),
    CY_beta(p.CY_beta),
    velocity_bodyFrame(p.velocity_bodyFrame),
    position_earthFrame(p.position_earthFrame), 
    k(p.k),
    tailArea(p.tailArea),
    CL_alpha_tailWing(p.CL_alpha_tailWing),
    CL0_tailWing(p.CL0_tailWing),
    CD0_tailWing(p.CD0_tailWing),
    k_tailWing(p.k_tailWing)
{}

void Forces::calcAerodynamicForce(PlaneParameters& p)
{
    // Calculate Total Airspeed (V) RMS x and z component
    double tas_xz = sqrt(velocity_bodyFrame.x * velocity_bodyFrame.x + velocity_bodyFrame.z * velocity_bodyFrame.z); // Important include in REPORT -> velocity component in the y (sideForce) does not induce lift, therefore it is not included in the resultant velocity that induces lift
    if (tas_xz < 0.1) tas_xz = 0.1;


    // Calculate Dynamic Pressure (Q)

    double Q = 0.5 * air_density * tas_xz * tas_xz;

    if (axis == "x" || axis == "z")
    {

        // Angle of Attack (Alpha) is the angle between the nose and the oncoming air in the X-Z plane.
        double alpha = atan2(velocity_bodyFrame.z, velocity_bodyFrame.x) + wingSettingAngle;

        // Aerodynamic stall limit (approx 15 degrees)
		double MAX_ALPHA = 0.261799; // 15 degrees in radians

        double clamped_alpha = (alpha > MAX_ALPHA) ? MAX_ALPHA : (alpha < -MAX_ALPHA ? -MAX_ALPHA : alpha);

        // Coefficients using linear approximation
        // Lift Coeficient
        double CL = CL0 + (CL_alpha * alpha);
        double CL_tailWing = CL0_tailWing + (CL_alpha_tailWing * alpha); // The tail AoA is roughly the same as the main wing AoA.
        // Drag Coefficient 
        double CD = CD0 + k * CL * CL;
        double CD_tailWing = CD0_tailWing + k_tailWing * CL_tailWing * CL_tailWing;

        // Resolve Lift and Drag into total Newtons
        // Positive = lifting tail up, Negative = pushing tail down
        double mainWingsLift = CL * Q * wingArea;
        double tailLift_signed = -(CL_tailWing * Q * tailArea);
        double Lift = mainWingsLift + tailLift_signed;

        double mainWingsDrag = CD * Q * wingArea;
        double tailDrag = CD_tailWing * Q * tailArea;
        double Drag = mainWingsDrag + tailDrag;


        if (axis == "x")
        {
            // Transform Lift/Drag from Wind Frame back to Body Frame X
            aerodynamicForceResultant = -Drag * cos(clamped_alpha) + Lift * sin(clamped_alpha);
        }
        else if (axis == "z")
        {
            // Transform Lift/Drag from Wind Frame back to Body Frame Z
            aerodynamicForceResultant = -Drag * sin(clamped_alpha) - Lift * cos(clamped_alpha);
        }
    }
    else if (axis == "y")
    {
        // Sideslip (Beta)
        double beta = atan2(velocity_bodyFrame.y, velocity_bodyFrame.x);
        double MAX_BETA = 0.261799;
        double clamped_beta = (beta > MAX_BETA) ? MAX_BETA : (beta < -MAX_BETA ? -MAX_BETA : beta);

        double CY = CY_beta * clamped_beta;
        double tas_xy = sqrt(velocity_bodyFrame.x * velocity_bodyFrame.x + velocity_bodyFrame.y * velocity_bodyFrame.y);

        Q = 0.5 * air_density * tas_xy * tas_xy;

        aerodynamicForceResultant = CY * Q * wingArea;

    }

}

void Forces::calcThrust(PlaneParameters& p, double dt) {

    actual_Thrust = p.actual_Thrust;


    // Thrust acts only on X axis
    if (axis != "x") {

        return;
    }

    if (fuelPercentage <= 0.0)
    {
        // do not calculate thrust
        return;
    }



    // Calculate the targeted Thrust at steady state.
    double targeted_thrust = maxThrust_at_sea_level * throttle * pow((air_density / air_density_sea_level), 0.7);
    double tau = 0;

    // Depending if we are accelerating or decelerating the value of the time consant tau changes.
    if (targeted_thrust > actual_Thrust) {
        // Accelerating: Tau = 0.5s
        tau = 1.2; // After 5 time constants (6s), we will be at 99% of the targeted thrust.*

    }
    else {
        // Decelerating: Tau = 0.3s
        tau = 0.3; // After 5 time constants (1.5s), we will be at 99% of the targeted thrust.*

    }

    double thrust_derivative = (targeted_thrust - actual_Thrust) / tau;

    // Update actual thrust using Euler integration
    p.actual_Thrust = integrateEuler(actual_Thrust, thrust_derivative, dt);

}

double Forces::summation_of_Forces(PlaneParameters& p)
{
    if (axis == "x") {
        return (aerodynamicForceResultant + p.actual_Thrust);
    }
    else {
        return aerodynamicForceResultant;
    }
}


double calcGravitationalAccelerationCONST(double latitude_deg, double altitude)
{
    // Convert latitude to radians
    double latitude_rad = latitude_deg * (PI / 180.0);

    // WGS84 Constants (elipsoid earth)
    const double g_equator = 9.7803253359; // Gravity at equator
    const double k = 0.00193185265241;     // Somigliana constant
    const double e2 = 0.00669437999014;    // Eccentricity squared
    const double a = 6378137.0;            // Semi-major axis (equatorial radius) in meters
    const double f = 1.0 / 298.257223563;          // Flattening f = (a-b)/b
    const double m = 0.003449786003;            // m = (omega^2 * a^2 * b) / GM, where omega is angular velocity of Earth, b is semi-minor axis, GM is standard gravitational parameter

    // Calculate gravity at sea level for this specific latitude (Somigliana Eq)
    double sin2_lat = sin(latitude_rad) * sin(latitude_rad);
    double g_sea_level = g_equator * ((1.0 + k * sin2_lat) / sqrt(1.0 - e2 * sin2_lat));

    double g_altitude = g_sea_level * (1.0 - (2.0 * altitude / a) * (1.0 + f + m - 2 * f * sin2_lat));

    return g_altitude;

}

// **** Global Physics Function ****

double getDynamicPressure(PlaneParameters& p) {
    double v_rms = sqrt(p.velocity_bodyFrame.x * p.velocity_bodyFrame.x + p.velocity_bodyFrame.y * p.velocity_bodyFrame.y + p.velocity_bodyFrame.z * p.velocity_bodyFrame.z);
    if (v_rms < 0.1) v_rms = 0.1;

    xyz geodetic = EcefToGeodetic(p.position_earthFrame);
    double altitude = geodetic.z; // Geodetic Z is actual altitude in meters

    if (altitude < 0) altitude = 0;

    // Atmospheric density exponential decay model
    double rho_sl = p.air_density_sea_level;
    double air_density = rho_sl * exp(-altitude / 9000.0);
    p.air_density = air_density;
    return 0.5 * air_density * v_rms * v_rms;
}

xyz getLinearBodyForces(PlaneParameters& p, double dt, double throttle) {
    p.throttle = throttle;

    Forces forcesX("x", p);
    Forces forcesY("y", p);
    Forces forcesZ("z", p);

    forcesX.calcAerodynamicForce(p);
    forcesX.calcThrust(p, dt);

    updateFuelConsumption(p, dt);

    forcesY.calcAerodynamicForce(p);
    forcesZ.calcAerodynamicForce(p);


    return { forcesX.summation_of_Forces(p), forcesY.summation_of_Forces(p), forcesZ.summation_of_Forces(p) };
}


xyz getRotationalTorques(PlaneParameters& p, double Q, double elevator_deg, double aileron_deg, double rudder_deg) {

	// Calculate Total Velocity RMS
    double v_rms = sqrt(p.velocity_bodyFrame.x * p.velocity_bodyFrame.x + p.velocity_bodyFrame.y * p.velocity_bodyFrame.y + p.velocity_bodyFrame.z * p.velocity_bodyFrame.z);
    if (v_rms < 1.0) v_rms = 1.0;

    double alpha = atan2(p.velocity_bodyFrame.z, p.velocity_bodyFrame.x) + p.wingSettingAngle;
    double beta = atan2(p.velocity_bodyFrame.y, p.velocity_bodyFrame.x);

    double elevator_rad = elevator_deg * PI / 180;
    double aileron_rad = aileron_deg * PI / 180;
    double rudder_rad = rudder_deg * PI / 180;

    double moment_roll = Q * p.S * p.b * ((p.CL_beta * beta) + ((p.b / (2 * v_rms)) * p.CL_p * p.omega.x) + (p.CL_delta_aileron * aileron_rad));
    double moment_pitch = Q * p.S * p.c * (((1.0 / v_rms) * p.CM_alpha * alpha) + ((p.c / (2 * v_rms)) * p.CM_q * p.omega.y) + (p.CM_delta_elevator * elevator_rad));
    double moment_yaw = Q * p.S * p.b * ((p.CN_beta * beta) + ((p.b / (2.0 * v_rms)) * p.CN_r * p.omega.z) + (p.CN_delta_rudder * rudder_rad));

    return { moment_roll, moment_pitch, moment_yaw };
}

void updateRotationalDynamics(PlaneParameters& p, xyz torques, double dt) {
    double p_rad = p.omega.x;
    double q_rad = p.omega.y;
    double r_rad = p.omega.z;

    double RHS_L = torques.x - (p.Izz - p.Iyy) * q_rad * r_rad + p.Ixz * p_rad * q_rad;
    double RHS_M = torques.y + (p.Ixx - p.Izz) * p_rad * r_rad + p.Ixz * (p_rad * p_rad - r_rad * r_rad);
    double RHS_N = torques.z - (p.Iyy - p.Ixx) * p_rad * q_rad + p.Ixz * q_rad * r_rad;

    double alpha_y = RHS_M / p.Iyy;

    double D = (p.Ixx * p.Izz) - (p.Ixz * p.Ixz);
    double alpha_x = (p.Izz * RHS_L + p.Ixz * RHS_N) / D;
    double alpha_z = (p.Ixz * RHS_L + p.Ixx * RHS_N) / D;

    p.omega.x = integrateEuler(p.omega.x, alpha_x, dt);
    p.omega.y = integrateEuler(p.omega.y, alpha_y, dt);
    p.omega.z = integrateEuler(p.omega.z, alpha_z, dt);

    double phi = p.angles.x;   // Roll
    double theta = p.angles.y; // Pitch

    // Pre-calculate trigonometric values
    double sin_phi = sin(phi), cos_phi = cos(phi);
    double cos_theta = cos(theta);
    double tan_theta = tan(theta);

    double phi_dot = p.omega.x + (sin_phi * tan_theta * p.omega.y) + (cos_phi * tan_theta * p.omega.z);
    double theta_dot = (cos_phi * p.omega.y) - (sin_phi * p.omega.z);

    // Safety check: Prevent divide by zero crash if pitched exactly 90 degrees (Gimbal lock)
    double psi_dot = 0.0;
    if (abs(cos_theta) > 0.0001) {
        psi_dot = (sin_phi / cos_theta * p.omega.y) + (cos_phi / cos_theta * p.omega.z);
    }

    p.angles.x = integrateEuler(p.angles.x, phi_dot, dt);
    p.angles.y = integrateEuler(p.angles.y, theta_dot, dt);
    p.angles.z = integrateEuler(p.angles.z, psi_dot, dt);
}

void updateLinearDynamics(PlaneParameters& p, xyz body_forces, double dt) {

    xyz geodetic = EcefToGeodetic(p.position_earthFrame);

    double mass = p.mass;
    double lat = geodetic.x * (PI / 180.0); // geodetic.x is Latitude in degrees, convert to rads
    double lon = geodetic.y * (PI / 180.0); // geodetic.y is Longitude in degrees, convert to rads

    double g = calcGravitationalAccelerationCONST(geodetic.x, geodetic.z);

    // Convert local forces to ECEF Earth map and add gravity
    xyz earthAxis_resultantForces = GetTotalForceECEF(
        body_forces, mass, lat, lon, p.angles.x, p.angles.y, p.angles.z, p.position_earthFrame, g
    );

    // Acceleration = Force / Mass (Newton's Second Law)
    double acceleration_x = earthAxis_resultantForces.x / mass;
    double acceleration_y = earthAxis_resultantForces.y / mass;
    double acceleration_z = earthAxis_resultantForces.z / mass;

    // Integrate for velocity and then for position
    p.velocity_earthFrame.x = integrateEuler(p.velocity_earthFrame.x, acceleration_x, dt);
    p.velocity_earthFrame.y = integrateEuler(p.velocity_earthFrame.y, acceleration_y, dt);
    p.velocity_earthFrame.z = integrateEuler(p.velocity_earthFrame.z, acceleration_z, dt);

    p.position_earthFrame.x = integrateEuler(p.position_earthFrame.x, p.velocity_earthFrame.x, dt);
    p.position_earthFrame.y = integrateEuler(p.position_earthFrame.y, p.velocity_earthFrame.y, dt);
    p.position_earthFrame.z = integrateEuler(p.position_earthFrame.z, p.velocity_earthFrame.z, dt);
}

void updatePhysics(PlaneParameters& p, double dt, double pilotThrottle, double elevator, double aileron, double rudder) {

    // Get the current Latitude and Longitude to find Local North
    xyz geodetic = EcefToGeodetic(p.position_earthFrame);
    double lat_rad = geodetic.x * (PI / 180.0);
    double lon_rad = geodetic.y * (PI / 180.0);

    // Two-step rotation function (ECEF -> NED -> Body)
    p.velocity_bodyFrame = GetBodyVelocities(
        p.velocity_earthFrame,
        lat_rad, lon_rad,
        p.angles.x, p.angles.y, p.angles.z
    );

    double Q = getDynamicPressure(p);

    xyz linearForces = getLinearBodyForces(p, dt, pilotThrottle);
    xyz torques = getRotationalTorques(p, Q, elevator, aileron, rudder);

    updateRotationalDynamics(p, torques, dt);
    updateLinearDynamics(p, linearForces, dt);
}


