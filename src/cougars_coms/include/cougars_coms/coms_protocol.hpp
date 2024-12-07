
#ifndef _COUGARS_COMS_PROTOCOL_
#define _COUGARS_COMS_PROTOCOL_


#include <cstdint>

namespace cougars_coms {


enum COUG_MSG_ID : uint8_t {
    EMPTY = 0x00,
    EMERGENCY_KILL = 0xFF,
    CONFIRM_EMERGENCY_KILL = 0xCF
};

struct EmergencyKill {
    COUG_MSG_ID msg_id = EMERGENCY_KILL;
}__attribute__((packed));

struct ConfirmEmergencyKill {
    COUG_MSG_ID msg_id = CONFIRM_EMERGENCY_KILL;
    bool success;
}__attribute__((packed));

}


#endif //_COUGARS_COMS_PROTOCOL_