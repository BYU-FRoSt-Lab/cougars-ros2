#include <string>

#ifndef _COUGARS_COMS_PROTOCOL_
#define _COUGARS_COMS_PROTOCOL_


#include <cstdint>

namespace cougars_coms {


enum COUG_MSG_ID : uint8_t {
    EMPTY = 0x00,

    VEHICLE_STATUS = 0x10,
    REQUEST_STATUS = 0x11,

    EMERGENCY_KILL = 0x40,
    CONFIRM_EMERGENCY_KILL = 0x41,

    EMERGENCY_SURFACE = 0x50,
    CONFIRM_EMERGENCY_SURFACE = 0x51,
};


struct EmergencyKill {
    COUG_MSG_ID msg_id = EMERGENCY_KILL;
}__attribute__((packed));

struct ConfirmEmergencyKill {
    COUG_MSG_ID msg_id = CONFIRM_EMERGENCY_KILL;
    bool success;
}__attribute__((packed));

struct EmergencySurface {
    COUG_MSG_ID msg = EMERGENCY_SURFACE;
}__attribute__((packed));

struct ConfirmEmergencySurface {
    COUG_MSG_ID msg_id = CONFIRM_EMERGENCY_SURFACE;
    bool success;
}__attribute__((packed));

struct RequestStatus {
    COUG_MSG_ID msg_id = REQUEST_STATUS;
}__attribute__((packed));

struct VehicleStatus {
    COUG_MSG_ID msg_id = VEHICLE_STATUS;
    uint8_t vehicle_id;
    uint8_t dvl_vel;
    bool dvl_running;
    uint8_t battery_voltage; // in mV
    uint8_t waypoint;
    bool gps_connection;
    bool leak_detection;
    int16_t x;
    int16_t y;
    uint8_t depth;
    uint8_t heading;

}__attribute__((packed));



} // cougars_coms
#endif //_COUGARS_COMS_PROTOCOL_