
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

    EMERGENCY_SURFACE = 0xEE,
    CONFIRM_EMERGENCY_SURFACE = 0xEC,

};


enum COUG_STATUS_CODE : uint8_t {
    READY = 0x1,
    IN_MISSION = 0x2,
    // NON_MOOS_MISSION = 0x3,
    EMERGENCY_STOPPED = 0x4,
    EMERGENCY_SURFACING = 0x5,
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

    COUG_STATUS_CODE status_code;

    uint8_t moos_waypoint;
    uint8_t moos_behavior_number;

    int16_t x;
    int16_t y;
    uint16_t depth;
    uint16_t heading;

}__attribute__((packed));


} // cougars_coms

inline std::ostream& operator<<(std::ostream& os, cougars_coms::COUG_STATUS_CODE code)
{
    switch(code) {
        case     cougars_coms::READY :                os << "READY";                      break;
        case     cougars_coms::IN_MISSION :           os << "IN_MISSION";                 break;
        // case     cougars_coms::NON_MOOS_MISSION :     os << "NON_MOOS_MISSION";           break;
        case     cougars_coms::EMERGENCY_STOPPED :    os << "EMERGENCY_STOPPED";          break;
        case     cougars_coms::EMERGENCY_SURFACING :  os << "EMERGENCY_SURFACING";        break;
        default:                        os << "Unknown cougUV status code"; break;
    };
    return os;
}

#endif //_COUGARS_COMS_PROTOCOL_