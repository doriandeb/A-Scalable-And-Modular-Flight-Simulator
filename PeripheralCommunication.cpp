#include "PeripheralCommunication.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <cstring>

// **** Network config ****
static const char* PI_IP = "192.168.1.11"; // Change to your Pi's actual IP
static const int   PI_PORT = 5005;           // Pi listens on this port
static const int   PC_PORT = 5006;           // PC listens on this port
// ****

FlightBridge::FlightBridge() {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "[BRIDGE] WSAStartup failed: " << WSAGetLastError() << "\n";
        return;
    }

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        std::cerr << "[BRIDGE] socket() failed: " << WSAGetLastError() << "\n";
        WSACleanup();
        return;
    }

    // Bind to PC_PORT so we can receive packets from the Pi
    sockaddr_in pc_addr{};
    pc_addr.sin_family = AF_INET;
    pc_addr.sin_addr.s_addr = INADDR_ANY;
    pc_addr.sin_port = htons(PC_PORT);

    if (bind(sock, reinterpret_cast<sockaddr*>(&pc_addr), sizeof(pc_addr)) == SOCKET_ERROR) {
        std::cerr << "[BRIDGE] bind() failed: " << WSAGetLastError() << "\n";
        closesocket(sock);
        WSACleanup();
        return;
    }

    // Non-blocking mode
    u_long nonBlocking = 1;
    ioctlsocket(sock, FIONBIO, &nonBlocking);

    // Cache Pi's address for sending
    pi_addr.sin_family = AF_INET;
    pi_addr.sin_port = htons(PI_PORT);
    inet_pton(AF_INET, PI_IP, &pi_addr.sin_addr);

    initialised = true;
    std::cout << "[BRIDGE] Ready — Pi at " << PI_IP << "\n";
}

FlightBridge::~FlightBridge() {
    if (initialised) {
        closesocket(sock);
        WSACleanup();
    }
}

// **** Send telemetry to Pi ****
void FlightBridge::sendTelemetry(double pitch, double roll, double heading,
    double altitude, double speed, double fuel) {
    if (!initialised) return;

    char buf[256];
    // Format: "pitch,roll,heading,altitude,speed,fuel"
    sprintf_s(buf, sizeof(buf),
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        pitch, roll, heading, altitude, speed, fuel);

    int sent = sendto(sock, buf, static_cast<int>(strlen(buf)), 0,
        reinterpret_cast<sockaddr*>(&pi_addr), sizeof(pi_addr));

    if (sent == SOCKET_ERROR)
        std::cerr << "[BRIDGE] sendTelemetry error: " << WSAGetLastError() << "\n";
}

// **** Receive hardware feedback from Pi ****
bool FlightBridge::receiveHardwareFeedback(PilotControls& controls) {
    if (!initialised) return false;

    char buf[1024];
    char latest[1024];
    int  latestLen = -1;

    sockaddr_in from{};
    int fromLen = sizeof(from);

    // Drain every pending packet — only the newest matters
    while (true) {
        int n = recvfrom(sock, buf, sizeof(buf) - 1, 0,
            reinterpret_cast<sockaddr*>(&from), &fromLen);
        if (n > 0) {
            latestLen = n;
            memcpy(latest, buf, n);
        }
        else {
            break; // WSAEWOULDBLOCK or error — queue empty
        }
    }

    if (latestLen <= 0)
        return false; // Nothing new this frame

    latest[latestLen] = '\0';

    // Parse "aileron,elevator,rudder,throttle,brake,flaps"
    // Matches Pi send format (cockpit21.py line 734) + throttle field
    std::string              msg(latest);
    std::stringstream        ss(msg);
    std::string              token;
    std::vector<std::string> parts;

    while (std::getline(ss, token, ','))
        parts.push_back(token);

    if (parts.size() < 6) {
        std::cerr << "[BRIDGE] Malformed packet (" << parts.size()
            << " fields): " << msg << "\n";
        return false;
    }

    try {
        controls.aileron = std::stod(parts[0]); // -1.0 to +1.0
        controls.elevator = std::stod(parts[1]); // -1.0 to +1.0
        controls.rudder = std::stod(parts[2]); // -1.0 to +1.0
        controls.throttle = std::stod(parts[3]); //  0.0 to +1.0
        controls.brake = std::stoi(parts[4]); // 0 or 1
        controls.flaps = std::stoi(parts[5]); // 0, 1, or 2
    }
    catch (const std::exception& e) {
        std::cerr << "[BRIDGE] Parse error: " << e.what() << " in: " << msg << "\n";
        return false;
    }

    return true;
}
