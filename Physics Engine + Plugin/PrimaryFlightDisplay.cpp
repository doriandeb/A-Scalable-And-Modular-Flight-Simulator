#include "Parameters.h"
#include <cmath>
#include <iostream>

//test headers
//#include <vector>
//#include "MathFunctions.h"

using namespace std;

void updateFuelConsumption(PlaneParameters& p, double dt) // passing the object p to update two variables within the updateFuelConsumption function
{
    
    // Check if the tanks are empty
    if (p.current_fuel_mass <= 0.0) {
        p.current_fuel_mass = 0.0;
        p.actual_Thrust = 0.0; // The engines shut down
        
    }
    else {
        // Calculate fuel burned this 
        double burn_rate = p.TSFC * p.actual_Thrust;

        // Subtract from the tank
        p.current_fuel_mass -= (burn_rate * dt);
    }

    // Update total plane mass
    p.mass = p.mass_planeOnly + p.current_fuel_mass;
    p.fuelPercentage = p.current_fuel_mass / p.max_fuel * 100; 
}


void getCompassHeading(PlaneParameters& p) {
    // yaw is already relative to local North because of NED rotation matrix
    double yaw_radians = p.angles.z;

    // Convert to Degrees
    double yaw_degrees = yaw_radians * (180.0 / PI);

    // Wrap to 0-360 degrees for the compass gauge
    p.heading = fmod(yaw_degrees, 360.0);
    if (p.heading < 0.0) {
        p.heading += 360.0;
    }

}

