#include <iostream>
#include <chrono>
#include <thread>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <fstream>

#include "Parameters.h"
#include "Physics.h"
#include "MathFunctions.h"
#include "PrimaryFlightDisplay.h"
#include "PeripheralCommunication.h"

using namespace std;


// Meters-to-feet conversion factor
static constexpr double M_TO_FEET = 3.2808399;


// Reads hardware inputs from the Pi, sends updated telemetry back, then sleeps
void updateInstrumentsAndReadInputs(FlightBridge& bridge, PilotControls& hwInput,
    double pitch_deg, double roll_deg, double heading_deg,
    double altitude_ft, double speed_ms, double fuelPercent)
{
    // Pull latest hardware inputs — non-blocking, controls are unchanged if nothing arrived
    if (bridge.receiveHardwareFeedback(hwInput)) {
        cout << "[INPUT] "
            << " brake=" << hwInput.brake
            << " flaps=" << hwInput.flaps
            << " aileron=" << - hwInput.aileron
            << " elev=" << - hwInput.elevator 
            << " rudder=" << hwInput.rudder
            << " throttle=" << hwInput.throttle
            << "\n";
    }

    // Push updated flight state to the Pi instrument displays
    bridge.sendTelemetry(pitch_deg, roll_deg, heading_deg, altitude_ft, speed_ms, fuelPercent);

    Sleep(33); 
}


int main()
{
    // **** Initialise the aircraft ****
    PlaneParameters b747;

    // ECEF spawn position
    //b747.position_earthFrame = { 4183850.0, 966750.0, 4701250.0 };
    b747.position_earthFrame = { 6388137.0, 0.0, 0.0 };
    b747.angles = { 0.0, 0.022, 0.0 };  // [rad]  Pointing North, level flight

    // Convert the desired initial body-frame velocity into ECEF
    getVelocitiesEarthFrame(b747, 200.0, 0.0, 0.0);

    // **** Simulation constants ****
    const double dt = 1.0 / 60.0;  // [s]  Physics time step (~16.6 ms)
    double simulationTime = 0.0;
    const double sampleTime = 0.1;
    double counter = 1.0;
    double endTime = 5*60+1;
    using clock = std::chrono::steady_clock;

        
    // **** Procedure initializations ****
    double throttleInput = 0.5;
    double elevator = 0.0;
    double aileron = 0.0;
    double rudder = 0.0;
    
    short trigger = 1;


    // **** Keyboard input (currently inactive — hardware joystick is used instead) ****
    /*
    cout << "\n=========================================\n";
    cout << "ENGINE RUNNING! CONTROL THE PLANE:\n";
    cout << "Pitch: W (Down) / S (Up)\n";
    cout << "Roll:  A (Left) / D (Right)\n";
    cout << "Yaw:   Q (Left) / E (Right)\n";
    cout << "Power: SHIFT (Up) / CTRL (Down)\n";
    cout << "=========================================\n\n";
    */
    
    
    FlightBridge  bridge;
    PilotControls hwInput{};  // zero-initialise all control axes

    // **** CSV SETUP ****
    std::ofstream csvFile("telemetry_data.csv");
    // Write the column headers to the first row
    //csvFile << "Time_s,Throttle_Command,Actual_Thrust_N,VelocityX_ms\n";
    csvFile << "Time_s,Roll,Pitch,Yaw\n";

    // **** Main simulation loop (target: 60 Hz) ****
    while (true)
    {
        auto frameStart = clock::now();
        

        // **** Keyboard input (currently inactive — hardware joystick is used instead) ****
        // Beginning of Keyboard Controls
        /*
        double elevator = 0.0;  // [deg]
        double aileron = 0.0;  // [deg]
        double rudder = 0.0;  // [deg]

        // Pitch (W/S)
        if (GetAsyncKeyState('W') & 0x8000) elevator = 5.0;
        if (GetAsyncKeyState('S') & 0x8000) elevator = -5.0;

        // Roll (A/D)
        if (GetAsyncKeyState('A') & 0x8000) aileron = -5.0;
        if (GetAsyncKeyState('D') & 0x8000) aileron = 5.0;

        // Yaw (Q/E)
        if (GetAsyncKeyState('Q') & 0x8000) rudder = -5.0;
        if (GetAsyncKeyState('E') & 0x8000) rudder = 5.0;

        // Throttle (Shift / Ctrl)
        if (GetAsyncKeyState(VK_SHIFT) & 0x8000) throttleInput += 0.01;
        if (GetAsyncKeyState(VK_CONTROL) & 0x8000) throttleInput -= 0.01;
        if (throttleInput > 1.0) throttleInput = 1.0;
        if (throttleInput < 0.0) throttleInput = 0.0;

        // Mechanical deflection limits for a heavy airliner [deg]
        const double MAX_ELEVATOR = 20.0;
        const double MAX_AILERON = 25.0;
        const double MAX_RUDDER = 30.0;

        elevator = max(-MAX_ELEVATOR, min(elevator, MAX_ELEVATOR));
        aileron = max(-MAX_AILERON, min(aileron, MAX_AILERON));
        rudder = max(-MAX_RUDDER, min(rudder, MAX_RUDDER));
        */
        // End of Keyboard Controls

        // **** Read hardware inputs ****
        // Scale normalised joystick range [-1, 1]
        ///*
        double elevator = hwInput.elevator * 1.0; 
        double aileron = hwInput.aileron * 1.0;  
        double rudder = hwInput.rudder * 1.0;                    
        throttleInput = hwInput.throttle;   
        //*/
        // End of Joystick inputs

        
        // **** Procedure applying a unit step input to the throttle (45 min end time, 1 sec sample time) ****

        // Start at zero throttle. After 2 seconds of simulation, throttle is set to 100%
        /*
        if (simulationTime < 2.0) {
            throttleInput = 0.0;
        }
        else {
            throttleInput = 1.0;
        }
        double elevator = 0.0;
        double aileron = 0.0;
        double rudder = 0.0;
        */

        // **** Procedure applying a short pulse input to the elevator (5 min end time, 0.1 sample time) ****    
        
        /*
        if (simulationTime >= 5.0 && simulationTime <= 7.0)
        {
            elevator = -1.0;
        }
        else
        {
            elevator = 0.0;
        }
        */
        
        // ── Procedure applying a step input to the aileron (5 min end time, 0.1 sample time) ────────────
        /*
        double roll_degrees_testing = b747.angles.x * 180 / PI;
        if (trigger == 1) {
            aileron = -5.0;
            if (roll_degrees_testing >= 5.0) {
                trigger = 0;
            }
        }
        else {
            aileron = 0.0; 
        }
        */
        // ── Procedure applying a rectangular pulse input to the rudder (5 min end time, 0.1 sample time) ────────────
        /*
        if (simulationTime >= 5.0 && simulationTime <= 7.0)
        {
            rudder = 5.0;
        }
        else
        {
            rudder = 0.0;
        }
        */
        // **** Physics step ****
        updatePhysics(b747, dt, throttleInput, elevator, aileron, rudder);

        // **** Convert ECEF state to geodetic for display & telemetry ****
        const double RAD_TO_DEG = 180.0 / PI;

        xyz geodetic = EcefToGeodetic(b747.position_earthFrame);
        double lat_deg = geodetic.x;
        double lon_deg = geodetic.y;
        double alt_m = geodetic.z;
        double alt_ft = alt_m * M_TO_FEET;

        double roll_deg = b747.angles.x * RAD_TO_DEG;
        double pitch_deg = b747.angles.y * RAD_TO_DEG;
        double yaw_deg = b747.angles.z * RAD_TO_DEG;

        // **** Console HUD ****
        // \033[6A moves the cursor up 6 lines so the HUD overwrites itself in place
        ///*
        cout << "\033[6A\r"
            << " ALT: " << alt_m << "\t | LON: " << lon_deg << "\t | LAT: " << lat_deg << "          \n"
            << " | V_x_body: " << b747.velocity_bodyFrame.x << "          \n"
            << " | V_y_body: " << b747.velocity_bodyFrame.y << "          \n"
            << " | V_z_body: " << b747.velocity_bodyFrame.z << "          \n"
            << " | PITCH : " << pitch_deg << "\t | ROLL : " << roll_deg << "\t | YAW : " << yaw_deg << "          \n"
            << " | HEADING: " << b747.heading << "          \n"
            << " | Throttle : " << throttleInput << "\t | FUEL : " << b747.fuelPercentage << "%          " << flush;
        //*/
        // **** Update compass heading and send telemetry to instruments ****
        getCompassHeading(b747);

        updateInstrumentsAndReadInputs(bridge, hwInput,
            pitch_deg, roll_deg, b747.heading,
            alt_ft, b747.velocity_bodyFrame.x,
            b747.fuelPercentage);

        // **** LOG THE DATA TO THE CSV FOR LINEAR DYNAMICS TEST ****
        /*
        if (counter >= sampleTime) {
            csvFile << simulationTime << ","
                << throttleInput << ","
                << b747.actual_Thrust << ","
                << alt_m << ","
                << b747.velocity_bodyFrame.x << "\n";
            counter = 0.0;
            cout << "Time: " << simulationTime << " / " << endTime << endl;
        }
        */
        // **** LOG THE DATA TO THE CSV FOR ROTATIONAL DYNAMICS TEST ****
        /*
        if (counter >= sampleTime) {
            csvFile << simulationTime << ","
                << roll_deg << ","         // Roll
                << pitch_deg << ","         // Pitch
                << yaw_deg << ","         // Yaw
                << "\n";
            counter = 0.0;

            cout << "Time: " << simulationTime << " / " << endTime;
            cout << "\t roll: " << roll_deg;
            cout << "\t pitch: " << pitch_deg;
            cout << "\t yaw: " << yaw_deg << endl;
            
        }
        */

        // Increment simulation time tracker
        simulationTime += dt;
        counter += dt;

        // **** Frame timing — sleep off any remaining time to hold 60 Hz ****
        auto frameEnd = clock::now();
        std::chrono::duration<double> elapsed = frameEnd - frameStart;
        double remaining = dt - elapsed.count();
        

        if (remaining > 0.0)
            std::this_thread::sleep_for(std::chrono::duration<double>(remaining));

        if (simulationTime > endTime)
            break;

    }
    
    csvFile.close();
    return 0;
}