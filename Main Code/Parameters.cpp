#include "Parameters.h"
#include <iostream>
#include <vector>

using namespace std;

// Constructor Implementation
PlaneParameters::PlaneParameters()
{

	mass = mass_planeOnly + current_fuel_mass;

	fuelPercentage = current_fuel_mass / max_fuel * 100; // %
}

