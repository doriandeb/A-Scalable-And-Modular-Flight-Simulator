#pragma once
#include <vector>

#define PI 3.14159265358979323846

using namespace std;

class PlaneParameters;

struct xyz {
    double x;
    double y;
    double z;
};

double integrateEuler(double y, double dy_dt, double dt);


xyz EcefToGeodetic(xyz cartesian);

xyz GeodeticToEcef(double lat_degrees, double lon_degrees, double alt_meters);

// **** Matrix-Vector Multiplication ****   
xyz RotateVector(double R[3][3], xyz v);

xyz GetTotalForceECEF(
    xyz body_forces,   // {Thrust-Drag, SideForce, Lift-Weight_z}
    double mass,
    double lat, double lon, // Radians
    double roll, double pitch, double yaw, // Radians
    xyz plane_pos_ecef, // Current X, Y, Z
    double g
);

void getVelocitiesEarthFrame(PlaneParameters& p, double u, double v, double w);

// Converts global ECEF velocity back into Local Body velocities (u, v, w)
xyz GetBodyVelocities(xyz vel_ecef, double lat, double lon, double roll, double pitch, double yaw);