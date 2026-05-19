#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>

//headers from X-Plane SDK
#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMGraphics.h" 
#include <string.h>
#include <cmath> 
#include "XPLMUtilities.h"
#include <XPLMDisplay.h>
#include <cstdio> 
#include <iostream>

//headers from this project
#include "Parameters.h"
#include "Physics.h"
#include "MathFunctions.h"
#include "PrimaryFlightDisplay.h"
#include "PeripheralCommunication.h"

using namespace std;

// Meters-to-feet conversion factor
static constexpr double M_TO_FEET = 3.2808399;

// Global Datarefs
XPLMDataRef g_override_planepath = NULL;
XPLMDataRef g_local_x = NULL;
XPLMDataRef g_local_y = NULL;
XPLMDataRef g_local_z = NULL;
XPLMDataRef g_phi = NULL;
XPLMDataRef g_theta = NULL;
XPLMDataRef g_psi = NULL;

// Global Instances
PlaneParameters MyPlane;
bool g_is_initialized = false;

// Hardware Bridge And Input State
FlightBridge g_bridge;
PilotControls g_hwInput{};

// Global Keyboard State Flags
bool key_W_down = false;
bool key_A_down = false;
bool key_S_down = false;
bool key_D_down = false;

// Function Prototypes
float PhysicsLoop(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon);

// Keyboard Sniffer 
// This listens to all keyboard traffic in X-Plane
int MyKeySniffer(char inChar, XPLMKeyFlags inFlags, char inVirtualKey, void* inRefcon) {
    if (inFlags & xplm_DownFlag) {
        if (inVirtualKey == XPLM_VK_W) key_W_down = true;
        if (inVirtualKey == XPLM_VK_S) key_S_down = true;
        if (inVirtualKey == XPLM_VK_A) key_A_down = true;
        if (inVirtualKey == XPLM_VK_D) key_D_down = true;
    }
    if (inFlags & xplm_UpFlag) {
        if (inVirtualKey == XPLM_VK_W) key_W_down = false;
        if (inVirtualKey == XPLM_VK_S) key_S_down = false;
        if (inVirtualKey == XPLM_VK_A) key_A_down = false;
        if (inVirtualKey == XPLM_VK_D) key_D_down = false;
    }
    return 1; // Return 1 to let X-Plane continue using these keys too
}



// **** Plugin Setup ****

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    strcpy(outName, "Custom Flight Physics");
    strcpy(outSig, "com.physics.custom.flight");
    strcpy(outDesc, "6-DOF Physics");

    // Datarefs
    g_override_planepath = XPLMFindDataRef("sim/operation/override/override_planepath");
    g_local_x = XPLMFindDataRef("sim/flightmodel/position/local_x");
    g_local_y = XPLMFindDataRef("sim/flightmodel/position/local_y");
    g_local_z = XPLMFindDataRef("sim/flightmodel/position/local_z");
    g_phi = XPLMFindDataRef("sim/flightmodel/position/phi");
    g_theta = XPLMFindDataRef("sim/flightmodel/position/theta");
    g_psi = XPLMFindDataRef("sim/flightmodel/position/psi");

    if (!g_override_planepath || !g_local_x || !g_local_y || !g_local_z) return 0;

    XPLMRegisterFlightLoopCallback(PhysicsLoop, -1.0, NULL);
    XPLMRegisterKeySniffer(MyKeySniffer, 1, NULL);

    return 1;
}

PLUGIN_API int XPluginEnable(void) {
    g_is_initialized = false;
    int override_array[1] = { 1 };
    XPLMSetDatavi(g_override_planepath, override_array, 0, 1);
    return 1;
}

PLUGIN_API void XPluginDisable(void) {
    int override_array[1] = { 0 };
    XPLMSetDatavi(g_override_planepath, override_array, 0, 1);
}

PLUGIN_API void XPluginStop(void) {
    XPLMUnregisterFlightLoopCallback(PhysicsLoop, NULL);
    XPLMUnregisterKeySniffer(MyKeySniffer, 1, NULL);
}


// **** Main Physics Loop (Runs every frame) ****


float PhysicsLoop(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void* inRefcon) {

    // Initilisation: Get current position
    if (!g_is_initialized) {
        double start_x = XPLMGetDatad(g_local_x);
        double start_y = XPLMGetDatad(g_local_y);
        double start_z = XPLMGetDatad(g_local_z);

        double lat, lon, alt;
        XPLMLocalToWorld(start_x, start_y, start_z, &lat, &lon, &alt);

        alt += 100.0;
        MyPlane.position_earthFrame = GeodeticToEcef(lat, lon, alt);

        MyPlane.angles.x = XPLMGetDataf(g_phi) * (3.14159265 / 180.0);
        MyPlane.angles.y = XPLMGetDataf(g_theta) * (3.14159265 / 180.0);
        MyPlane.angles.z = XPLMGetDataf(g_psi) * (3.14159265 / 180.0);

		//setting the initial velocity to 250 m/s forward
        getVelocitiesEarthFrame(MyPlane, 250.0, 0.0, 0.0);
        MyPlane.omega = { 0, 0, 0 };

        g_is_initialized = true;
        return -1.0f;
    }

 
    // Run the Physics Engine
    double dt = inElapsedSinceLastCall;
    if (dt > 0.1) dt = 0.1;

	//Initial control inputs (will be overwritten by hardware or keyboard)
    double throttleInput = 1.0;
    double elevator = 0.0;
    double aileron = 0.0;
    double rudder = 0.0;

    // Hardware and WASD Inputs
    // First, check for keyboard input (WASD fallback)
    if (key_W_down) elevator = -5.0; // W = Push Nose Down
    if (key_S_down) elevator = 5.0;  // S = Pull Nose Up
    if (key_A_down) aileron = -5.0;  // A = Roll Left
    if (key_D_down) aileron = 5.0;   // D = Roll Right

    // Next, poll the Pi hardware. If hardware is sending data, it overwrites the keyboard.
    if (g_bridge.receiveHardwareFeedback(g_hwInput)) {
        elevator = - g_hwInput.elevator * 5.0;
        aileron = - g_hwInput.aileron * 5.0;
        rudder = g_hwInput.rudder * 5.0;
        throttleInput = g_hwInput.throttle;

    }

    // Passes the control inputs dynamically into the engine
    updatePhysics(MyPlane, dt, throttleInput, elevator, aileron, rudder);

    
    // Telemtry AND PI Updates
    xyz geodetic = EcefToGeodetic(MyPlane.position_earthFrame);
    double alt_ft = geodetic.z * M_TO_FEET;
    double roll_deg = MyPlane.angles.x * (180.0 / 3.14159265);
    double pitch_deg = MyPlane.angles.y * (180.0 / 3.14159265);

    getCompassHeading(MyPlane);

    // Pushes updated flight state to the Pi instrument displays
    g_bridge.sendTelemetry(
        pitch_deg,
        roll_deg,
        MyPlane.heading,
        alt_ft,
        MyPlane.velocity_bodyFrame.x,
        MyPlane.fuelPercentage
    );

    // **** Anti-Crash Shield ****
    if (std::isnan(MyPlane.position_earthFrame.x) || std::isnan(MyPlane.position_earthFrame.y) || std::isnan(MyPlane.position_earthFrame.z) ||
        std::isinf(MyPlane.position_earthFrame.x) || std::isinf(MyPlane.position_earthFrame.y) || std::isinf(MyPlane.position_earthFrame.z) ||
        std::isnan(MyPlane.velocity_earthFrame.x) || std::isnan(MyPlane.velocity_earthFrame.y) || std::isnan(MyPlane.velocity_earthFrame.z) ||
        std::isinf(MyPlane.velocity_earthFrame.x) || std::isinf(MyPlane.velocity_earthFrame.y) || std::isinf(MyPlane.velocity_earthFrame.z))
    {
        
        MyPlane.velocity_earthFrame = { 0.0, 0.0, 0.0 };
        MyPlane.omega = { 0.0, 0.0, 0.0 };
        return -1.0f;
    }

    // **** Export Coordinates to X-Plane ****

    double new_local_x, new_local_y, new_local_z;
    XPLMWorldToLocal(geodetic.x, geodetic.y, geodetic.z, &new_local_x, &new_local_y, &new_local_z);

    XPLMSetDatad(g_local_x, new_local_x);
    XPLMSetDatad(g_local_y, new_local_y);
    XPLMSetDatad(g_local_z, new_local_z);

    XPLMSetDataf(g_phi, (float)roll_deg);
    XPLMSetDataf(g_theta, (float)pitch_deg);
    XPLMSetDataf(g_psi, (float)(MyPlane.angles.z * (180.0 / 3.14159265)));

    return -1.0f;
}