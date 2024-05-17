


// types of commands that can be sent to a coug uv using
// the seatrac modem.
// It only has EMERGENCY_STOP right now but we can add
// more commands later
// These are the first bytes in the payload of a CID_DAT_SEND command.
enum COUG_UV_SEATRAC_COMMANDS {
    EMERGENCY_STOP = 0x45,
};

