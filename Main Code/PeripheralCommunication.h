#ifndef PERIPHERAL_COMMUNICATION_H
#define PERIPHERAL_COMMUNICATION_H

#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

struct PilotControls {
    double aileron = 0.0;  // -1.0 (left)    to +1.0 (right)
    double elevator = 0.0;  // -1.0 (push)     to +1.0 (pull)
    double rudder = 0.0;  // -1.0 (left)     to +1.0 (right)
    double throttle = 0.0;  //  0.0 (idle)      to +1.0 (full)
    int    brake = 0;    // 0 = off, 1 = on
    int    flaps = 0;    // 0 = up, 1 = half, 2 = full
};

class FlightBridge {
private:
    SOCKET      sock;
    sockaddr_in pi_addr{};
    bool        initialised = false;

public:
    FlightBridge();
    ~FlightBridge();

    // Send physics state to Pi instruments
    void sendTelemetry(double pitch, double roll, double heading,
        double altitude, double speed, double fuel);

    // Read latest hardware inputs from Pi (non-blocking, keeps newest packet)
    bool receiveHardwareFeedback(PilotControls& controls);
};

#endif