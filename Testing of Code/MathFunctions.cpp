#include "MathFunctions.h"
#include "Parameters.h"
#include <cmath>

using namespace std;



// **** Euler integration: advances value y by one time step ****
double integrateEuler(double y, double dy_dt, double dt)
{
    return y + dy_dt * dt;
}


// Converts geodetic coordinates to ECEF Cartesian (X, Y, Z) using WGS-84.
xyz GeodeticToEcef(double lat_deg, double lon_deg, double alt_m)
{
    // WGS-84 ellipsoid constants
    const double a = 6378137.0;          // [m]  Semi-major axis (equatorial radius)
    const double e2 = 0.00669437999014;   // [-]  First eccentricity squared

    const double lat_rad = lat_deg * (PI / 180.0);
    const double lon_rad = lon_deg * (PI / 180.0);

    const double sin_lat = sin(lat_rad);
    const double cos_lat = cos(lat_rad);
    const double sin_lon = sin(lon_rad);
    const double cos_lon = cos(lon_rad);

    // Prime vertical radius of curvature at this latitude
    const double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);

    xyz ecef;
    ecef.x = (N + alt_m) * cos_lat * cos_lon;
    ecef.y = (N + alt_m) * cos_lat * sin_lon;
    ecef.z = (N * (1.0 - e2) + alt_m) * sin_lat;

    return ecef;
}


// Converts ECEF Cartesian (X, Y, Z) back to geodetic using the Bowring iterative method.
// Returns xyz where: x = latitude [deg], y = longitude [deg], z = altitude [m]
xyz EcefToGeodetic(xyz ecef_pos)
{
    const double x = ecef_pos.x;
    const double y = ecef_pos.y;
    const double z = ecef_pos.z;

    // WGS-84 ellipsoid constants
    const double a = 6378137.0;
    const double e2 = 0.00669437999014;

    const double semi_minor_axis = sqrt(a * a * (1.0 - e2));
    const double e2_second = (a * a - semi_minor_axis * semi_minor_axis) / (semi_minor_axis * semi_minor_axis);  // Second eccentricity squared
    const double dist_xy = sqrt(x * x + y * y);                                                               // Distance from Earth's rotation axis
    const double geocentric_lat = atan2(a * z, semi_minor_axis * dist_xy);                                           // Initial geocentric latitude estimate

    const double lon = atan2(y, x);
    const double lat = atan2(
        z + e2_second * semi_minor_axis * pow(sin(geocentric_lat), 3),
        dist_xy - e2 * a * pow(cos(geocentric_lat), 3)
    );

    const double N = a / sqrt(1.0 - e2 * sin(lat) * sin(lat));
    const double alt = dist_xy / cos(lat) - N;

    return { lat * (180.0 / PI), lon * (180.0 / PI), alt };
}


// Applies a 3×3 rotation matrix R to vector v.
xyz RotateVector(double R[3][3], xyz v)
{
    xyz result;
    result.x = R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z;
    result.y = R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z;
    result.z = R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z;
    return result;
}


// Rotates body-frame aerodynamic forces into ECEF, then adds the ECEF gravity vector.
// lat, lon, roll, pitch, yaw are all in radians.
xyz GetTotalForceECEF(
    xyz    body_forces,
    double mass,
    double lat, double lon,
    double roll, double pitch, double yaw,
    xyz    position_ecef,
    double g)
{
    // Body → NED rotation matrix (ZYX Euler convention)
    const double cR = cos(roll), sR = sin(roll);
    const double cP = cos(pitch), sP = sin(pitch);
    const double cY = cos(yaw), sY = sin(yaw);

    
    double R_body_to_ned[3][3] = {
        {  cP * cY,  sR * sP * cY - cR * sY,  cR * sP * cY + sR * sY },
        {  cP * sY,  sR * sP * sY + cR * cY,  cR * sP * sY - sR * cY },
        { -sP,       sR * cP,                 cR * cP                }
    };

    // NED → ECEF rotation matrix
    const double cLat = cos(lat), sLat = sin(lat);
    const double cLon = cos(lon), sLon = sin(lon);

    double R_ned_to_ecef[3][3] = {
        { -sLat * cLon,  -sLon,  -cLat * cLon },
        { -sLat * sLon,   cLon,  -cLat * sLon },
        {  cLat,          0.0,   -sLat        }
    };

    xyz aero_forces_ecef = RotateVector(R_ned_to_ecef, RotateVector(R_body_to_ned, body_forces));

    // Gravity always points toward Earth's centre in ECEF
    double radial_dist = sqrt(position_ecef.x * position_ecef.x +
        position_ecef.y * position_ecef.y +
        position_ecef.z * position_ecef.z);
    if (radial_dist == 0.0) radial_dist = 1.0;  // Avoid division by zero at the origin

    xyz gravity_ecef = {
        -(position_ecef.x / radial_dist) * mass * g,
        -(position_ecef.y / radial_dist) * mass * g,
        -(position_ecef.z / radial_dist) * mass * g
    };

    return {
        aero_forces_ecef.x + gravity_ecef.x,
        aero_forces_ecef.y + gravity_ecef.y,
        aero_forces_ecef.z + gravity_ecef.z
    };
}


// Converts an ECEF velocity vector into body-frame velocity components (u, v, w).
// All angles in radians. Two-step: ECEF -> NED -> Body.
xyz GetBodyVelocities(xyz vel_ecef, double lat, double lon, double roll, double pitch, double yaw)
{
    // Step 1: ECEF -> NED  (transpose of the NED→ECEF matrix)
    const double cLat = cos(lat), sLat = sin(lat);
    const double cLon = cos(lon), sLon = sin(lon);

    const double v_north = vel_ecef.x * (-sLat * cLon) + vel_ecef.y * (-sLat * sLon) + vel_ecef.z * (cLat);
    const double v_east = vel_ecef.x * (-sLon) + vel_ecef.y * (cLon);
    const double v_down = vel_ecef.x * (-cLat * cLon) + vel_ecef.y * (-cLat * sLon) + vel_ecef.z * (-sLat);

    // Step 2: NED -> Body  (transpose of the Body→NED matrix)
    const double cR = cos(roll), sR = sin(roll);
    const double cP = cos(pitch), sP = sin(pitch);
    const double cY = cos(yaw), sY = sin(yaw);

    const double u = v_north * (cP * cY) + v_east * (cP * sY) + v_down * (-sP);
    const double v = v_north * (sR * sP * cY - cR * sY) + v_east * (sR * sP * sY + cR * cY) + v_down * (sR * cP);
    const double w = v_north * (cR * sP * cY + sR * sY) + v_east * (cR * sP * sY - sR * cY) + v_down * (cR * cP);

    return { u, v, w };
}


// Sets the aircraft's initial ECEF velocity from a desired body-frame velocity (u, v, w).
// Called once at startup — converts body-frame intent into the ECEF state the integrator uses.
void getVelocitiesEarthFrame(PlaneParameters& p, double u, double v, double w)
{
    const xyz body_vel = { u, v, w };

    // Get the spawn lat/lon in radians (EcefToGeodetic returns degrees)
    const xyz startGeodetic = EcefToGeodetic(p.position_earthFrame);
    const double lat = startGeodetic.x * (PI / 180.0);
    const double lon = startGeodetic.y * (PI / 180.0);

    const double roll = p.angles.x;
    const double pitch = p.angles.y;
    const double yaw = p.angles.z;

    const double cR = cos(roll), sR = sin(roll);
    const double cP = cos(pitch), sP = sin(pitch);
    const double cY = cos(yaw), sY = sin(yaw);

    // Body → NED rotation matrix
    double R_body_to_ned[3][3] = {
        {  cP * cY,                   sR * sP * cY - cR * sY,   cR * sP * cY + sR * sY },
        {  cP * sY,                   sR * sP * sY + cR * cY,   cR * sP * sY - sR * cY },
        { -sP,                        sR * cP,                   cR * cP                }
    };

    const double cLat = cos(lat), sLat = sin(lat);
    const double cLon = cos(lon), sLon = sin(lon);

    // NED → ECEF rotation matrix
    double R_ned_to_ecef[3][3] = {
        { -sLat * cLon,  -sLon,  -cLat * cLon },
        { -sLat * sLon,   cLon,  -cLat * sLon },
        {  cLat,          0.0,   -sLat        }
    };

    p.velocity_earthFrame = RotateVector(R_ned_to_ecef, RotateVector(R_body_to_ned, body_vel));
}