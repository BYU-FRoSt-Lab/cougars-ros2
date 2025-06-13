
#ifndef _COUGARS_COMS_PROTOCOL_
#define _COUGARS_COMS_PROTOCOL_


#include <cstdint>

namespace cougars_coms {


enum COUG_MSG_ID : uint8_t {
    EMPTY = 0x00,

    VEHICLE_STATUS = 0x10,
    REQUEST_STATUS = 0x11,

    VERIFY_LAUNCH = 0x20,        
    CONFIRM_VERIFY = 0x21,

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

    uint8_t waypoint;

    uint8_t battery_voltage;
    uint8_t battery_percentage;

    uint8_t leak;

    uint8_t safety_mask;

    int8_t x;
    int8_t y;
    int8_t x_vel;
    int8_t y_vel;
    uint8_t depth;
    uint8_t heading;

}__attribute__((packed));



} // cougars_coms
#endif //_COUGARS_COMS_PROTOCOL_