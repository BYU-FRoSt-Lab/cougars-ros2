
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

    START_MISSION = 0x30,
    CONFIRM_START_MISSION = 0x31,

    EMERGENCY_KILL = 0xFF,
    CONFIRM_EMERGENCY_KILL = 0xFC,

};


struct EmergencyKill {
    static const COUG_MSG_ID msg_id = EMERGENCY_KILL;
}__attribute__((packed));

struct ConfirmEmergencyKill {
    static const COUG_MSG_ID msg_id = CONFIRM_EMERGENCY_KILL;
    bool success;
}__attribute__((packed));




struct VerifyLaunch {
    static const COUG_MSG_ID msg_id = VERIFY_LAUNCH;
}__attribute__((packed));

struct ConfirmVerifyLaunch {
    static const COUG_MSG_ID msg_id = CONFIRM_VERIFY;
    bool ready;
}__attribute__((packed));




struct StartMission {
    static const COUG_MSG_ID msg_id = START_MISSION;
    uint32_t start_time;
}__attribute__((packed));

struct ConfirmStartMission {
    static const COUG_MSG_ID msg_id = CONFIRM_START_MISSION;
    bool success;
}__attribute__((packed));




struct RequestStatus {
    static const COUG_MSG_ID msg_id = REQUEST_STATUS;
}__attribute__((packed));

struct VehicleStatus {
    static const COUG_MSG_ID msg_id = VEHICLE_STATUS;

    uint32_t timestamp;

    uint8_t moos_waypoints_reached; //last bit indicates that the waypoint is reached
    uint8_t moos_behavior_number;   
    uint8_t moos_error_code;

    int16_t x;
    int16_t y;
    uint16_t depth;
    uint16_t heading;

}__attribute__((packed));



} // cougars_coms
#endif //_COUGARS_COMS_PROTOCOL_