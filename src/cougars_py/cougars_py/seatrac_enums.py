
 #========================================================================#
 #      This file contains definitions of variables used to send          #
 #      and interpret ros messages from the seatrac modem                 #
 #========================================================================#


 # all definitions included in this file are defined in this manual
 # https://www.seascapesubsea.com/downloads/Blueprint-Subsea-SeaTrac-Developer-Guide.pdf


class SeatracEnum:
    # Helper method to get the name of each value in the enum
    @classmethod
    def to_str(cls, value:int):
        for name, atr_value in cls.__dict__.items():
            if atr_value == value:
                return name
        return "UNDEFINED"


 # BID_E : Beacon Identification Code
 #
 # Beacon Identification (BID) Codes are used to identify a specific beacon
 # that should receive acoustic messages or identify which beacon was the
 # source (sender) of a message. Valid values are in the range from 0 to 15 and
 # are typically send and stored as a UINT8.
class BID_E(SeatracEnum):
    BEACON_ALL = 0x0    # When used as an address for sending acoustic
                        # messages to the value of 0x00 indicates "broadcast
                        # to all".  When used as an identifier of a sender of
                        # a message the value of 0x00 should be interpreted
                        # as unknown or invalid (reserved).
    BEACON_ID_1  = 0x1
    BEACON_ID_2  = 0x2
    BEACON_ID_3  = 0x3
    BEACON_ID_4  = 0x4
    BEACON_ID_5  = 0x5
    BEACON_ID_6  = 0x6
    BEACON_ID_7  = 0x7
    BEACON_ID_8  = 0x8
    BEACON_ID_9  = 0x9
    BEACON_ID_10 = 0xa
    BEACON_ID_11 = 0xb
    BEACON_ID_12 = 0xc
    BEACON_ID_13 = 0xd
    BEACON_ID_14 = 0xe
    BEACON_ID_15 = 0xf



 # CID_E : Command Identification Codes
 #
 # Command Identification (CID) Codes are an classeration (or defined set of
 # constants) stored/transmitted in a UINT8 type at the start of Command and
 # Response messages after the synchronisation character with the purpose of
 # identifying the message function and its payload.
 #
 # only a subset of CID codes are included here. Full list can be found in 
 # the seatrac manual linked above 
 #
class CID_E(SeatracEnum):
     # PING Protocol Messages
    CID_PING_SEND = 0x40   # Command sent to transmit a PING message.
    CID_PING_REQ = 0x41    # Message sent when a PING request is received.
    CID_PING_RESP = 0x42   # Message sent when a PING response is received or
                           # timeout occurs with the echo response data.
    CID_PING_ERROR = 0x43  # Message sent when a PING response error/timeout
                           # occurs.

    # ECHO Protocol Messages
    CID_ECHO_SEND = 0x48   # Command sent to transmit an ECHO message.
    CID_ECHO_REQ = 0x49    # Message sent when an ECHO request is received.
    CID_ECHO_RESP = 0x4A   # Message sent when an ECHO response is received
                           # or timeout occurs with the echo response data.
    CID_ECHO_ERROR = 0x4B  # Message sent when an ECHO response error/timeout
                           # occurs.

    # DAT Protocol Messages
    CID_DAT_SEND = 0x60          # Message sent to transmit a datagram to
                                 # another beacon
    CID_DAT_RECEIVE = 0x61       # Message generated when a beacon receives a
                                 # datagram.
    CID_DAT_ERROR = 0x63         # Message generated when a beacon response
                                 # error/timeout occurs for ACKs.
    CID_DAT_QUEUE_SET = 0x64     # Message sent to set the contents of the
                                 # packet data queue.
    CID_DAT_QUEUE_CLR = 0x65     # Message sent to clear the contents of the
                                 # packet data queue.
    CID_DAT_QUEUE_STATUS = 0x66  # Message sent to obtain the current status
                                 # of the packet data queue.

    # NAV Protocol Messages
    CID_NAV_QUERY_SEND = 0x50      # Message sent to query navigation
                                   # information from a remote beacon.
    CID_NAV_QUERY_REQ = 0x51       # Message sent from a beacon that receives
                                   # a NAV_QUERY.
    CID_NAV_QUERY_RESP = 0x52      # Message generated when the beacon
                                   # received a response to a NAV_QUERY.
    CID_NAV_ERROR = 0x53           # Message generated if there is a problem
                                   # with a NAV_QUERY - i.e. timeout etc.

    # Status Messages
    CID_STATUS = 0x10          # Command sent to request the current system
                               # status (AHRS, Depth, Temp, etc).
    CID_STATUS_CFG_GET = 0x11  # Command sent to retrieve the configuration of
                               # the status system (message content and
                               # auto-output interval).
    CID_STATUS_CFG_SET = 0x12  # Command sent to set the configuration of the
                               # status system (message content and
                               # auto-output interval).

    # Acoustic Transceiver Messages
    CID_XCVR_ANALYSE = 0x30       # Command sent to instruct the receiver to
                                  # perform a noise analysis and report the
                                  # results.
    CID_XCVR_TX_MSG = 0x31        # Message sent when the transceiver
                                  # transmits a message.
    CID_XCVR_RX_ERR = 0x32        # Message sent when the transceiver receiver
                                  # encounters an error.
    CID_XCVR_RX_MSG = 0x33        # Message sent when the transceiver receives
                                  # a message (not requiring a response).
    CID_XCVR_RX_REQ = 0x34        # Message sent when the transceiver receives
                                  # a request (requiring a response).
    CID_XCVR_RX_RESP = 0x35       # Message sent when the transceiver receives
                                  # a response (to a transmitted request).
    CID_XCVR_RX_UNHANDLED = 0x37  # Message sent when a message has been
                                  # received but not handled by the protocol
                                  # stack.
    CID_XCVR_USBL = 0x38          # Message sent when a USBL signal is decoded
                                  # into an angular bearing.
    CID_XCVR_FIX = 0x39           # Message sent when the transceiver gets a
                                  # position/range fix on a beacon from a
                                  # request/response.
    CID_XCVR_STATUS = 0x3A        # Message sent to query the current
                                  # transceiver state.


 # AMSGTYPE_E : Acoustic Message Type.
 #
 # The AMSGTYPE_E classeration is used to specify a type of acoustic message
 # and determines how the message is processed and which responses are
 # generated from beacons.
class AMSGTYPE_E(SeatracEnum):
    MSG_OWAY = 0x0     # Indicates an acoustic message is sent One-Way and
                       # does not require a response. One-Way messages may
                       # also be broadcast to all beacons if required.  No
                       # USBL information is sent.
    MSG_OWAYU = 0x1    # Indicates an acoustic message is sent One-Way and
                       # does not require a response. One-Way messages may
                       # also be broadcast to all beacons if required.
                       # Additionally the message is sent with USBL acoustic
                       # information allowing an incoming bearing to be
                       # determined by USBL receivers although range cannot
                       # be provided.
    MSG_REQ = 0x2      # Indicates an acoustic message is sent as a Request
                       # type.  This requires the receiver to generate and
                       # return a Response (MSG_RESP) message.  No USBL
                       # information is requested.
    MSG_RESP = 0x3     # Indicate an acoustic message is sent as a Response to
                       # a previous Request message (MSG_REQ).  No USBL
                       # information is returned.
    MSG_REQU = 0x4     # Indicates an acoustic message is sent as a Request
                       # type.  This requires the receiver to generate and
                       # return a Response (MSG_RESP) message.  Additionally
                       # the Response message should be returned with USBL
                       # acoustic information allowing a position fix to be
                       # computed by USBL receivers through the range and
                       # incoming signal angle.
    MSG_RESPU = 0x5    # Indicate an acoustic message is sent as a Response to
                       # a previous Request message (MSG_REQ).  Additionally
                       # the message is sent with USBL acoustic information
                       # allowing the position of the sender to be determined
                       # through the range and incoming signal angle.
    MSG_REQX = 0x6     # Indicates an acoustic message is sent as a Request
                       # type.  This requires the receiver to generate and
                       # return a Response (MSG_RESP) message.  Additionally
                       # the Response message should be returned with extended
                       # Depth and USBL acoustic information allowing a more
                       # accurate position fix to be computed by USBL
                       # receivers through the range remote depth and
                       # incoming signal angle.
    MSG_RESPX = 0x7    # Indicate an acoustic message is sent as a Response to
                       # a previous Request message (MSG_REQ).  Additionally
                       # the message is sent with extended depth and USBL
                       # acoustic information allowing a more accurate
                       # position of the sender to be determined through the
                       # range remote depth and incoming signal angle.
    MSG_UNKNOWN = 0xFF # This value is NEVER used to specify a message type
                       # when sending Acoustic Messages. However on occasions
                       # certain structures need to specify "No Message Type"
                       # (for example see ACOFIX_T) and this value is used as
                       # an OUTPUT ONLY to indicate this.

class NAV_QUERY_E(SeatracEnum):
    QRY_DEPTH = 0x01     # When set, a NAV_QUERY_SEND command will request
                         # that depth information is sent back, and a
                         # NAV_QUERY_RESP will contain depth data fields.
    QRY_SUPPLY = 0x02    # When set, a NAV_QUERY_SEND command will request
                         # that supply voltage information is sent back, and a
                         # NAV_QUERY_RESP will contain supply voltage data
                         # fields.
    QRY_TEMP = 0x04      # When set, a NAV_QUERY_SEND command will request
                         # that temperature information is sent back, and a
                         # NAV_QUERY_RESP will contain temperature data
                         # fields.
    QRY_ATTITUDE = 0x08  # When set, a NAV_QUERY_SEND command will request
                         # that attitude information is sent back, and a
                         # NAV_QUERY_RESP will contain attitude data fields.
    QRY_DATA = 0x80      # When set, a NAV_QUERY_SEND command will request
                         # that any queued pending NAV data should be sent
                         # back, and a NAV_QUERY_RESP will contain data
                         # payload fields.



# CST_E : Command Status Codes
# Command Status (CST) Codes are an enumeration (or set of defined constants)
# that are commonly used in Response  messages sent from the beacon to
# indicate if a command executed successfully,  or if not, what type of error
# occurred.  CST codes are always transmitted as a  UINT8 type.
# Different Response messages may only implement a subset of the constants
# below, as appropriate for their function.
class CST_E(SeatracEnum):
    # General Status Codes
    CST_OK = 0x00            # Returned if a command or operation is completed
                             # successful without error.
    CST_FAIL = 0x01          # Returned if a command or operation cannot be
                             # completed.
    CST_EEPROM_ERROR = 0x03  # Returned if an error occurs while reading or
                             # writing EEPROM data.

    # Command Processor Status Codes
    CST_CMD_PARAM_MISSING = 0x04  # Returned if a command message is given
                                  # that does not have enough defined fields
                                  # for the specified CID code.
    CST_CMD_PARAM_INVALID = 0x05  # Returned if a data field in a message does
                                  # not contain a valid or expected value.

    # Firmware Programming Status Codes
    CST_PROG_FLASH_ERROR = 0x0A     # Returned if an error occurs while
                                    # writing data into the processors flash
                                    # memory.
    CST_PROG_FIRMWARE_ERROR = 0x0B  # Returned if firmware cannot be
                                    # programmed due to incorrect firmware
                                    # credentials or signature.
    CST_PROG_SECTION_ERROR = 0x0C   # Returned if the firmware cannot be
                                    # programmed into the specified memory
                                    # section.
    CST_PROG_LENGTH_ERROR = 0x0D    # Returned if the firmware length is too
                                    # large to fit into the specified memory
                                    # section, or not what the current
                                    # operation is expecting.
    CST_PROG_DATA_ERROR = 0x0E      # Returned if there is an error decoding
                                    # data in a firmware block.
    CST_PROG_CHECKSUM_ERROR = 0x0F  # Returned if the specified checksum for
                                    # the firmware does not match the checksum
                                    # computed prior to performing the update.

    # Acoustic Transceiver Status Codes
    CST_XCVR_BUSY = 0x30           # Returned if the transceiver cannot
                                   # perform a requested action as it is
                                   # currently busy (i.e. transmitting a
                                   # message).
    CST_XCVR_ID_REJECTED = 0x31    # Returned if the received message did not
                                   # match the specified transceiver ID (and
                                   # wasn t a Sent-To-All), and the message
                                   # has been rejected.
    CST_XCVR_CSUM_ERROR = 0x32     # Returned if received acoustic message's
                                   # checksum was invalid, and the message has
                                   # been rejected.
    CST_XCVR_LENGTH_ERROR = 0x33   # Returned if an error occurred with
                                   # message framing, meaning the end of the
                                   # message has not been received within the
                                   # expected time.
    CST_XCVR_RESP_TIMEOUT = 0x34   # Returned if the transceiver has sent a
                                   # request message to a beacon, but no
                                   # response has been returned within the
                                   # allotted waiting period.
    CST_XCVR_RESP_ERROR = 0x35     # Returned if the transceiver has send a
                                   # request message to a beacon, but an error
                                   # occurred while receiving the response.
    CST_XCVR_RESP_WRONG = 0x36     # Returned if the transceiver has sent a
                                   # request message to a beacon, but received
                                   # an unexpected response from another
                                   # beacon while waiting.
    CST_XCVR_PLOAD_ERROR = 0x37    # Returned by protocol payload decoders, if
                                   # the payload can't be parsed correctly.
    CST_XCVR_STATE_STOPPED = 0x3A  # Indicates the transceiver is in a stopped state.
    CST_XCVR_STATE_IDLE = 0x3B     # Indicates the transceiver is in an idle
                                   # state waiting for reception or
                                   # transmission to start.
    CST_XCVR_STATE_TX = 0x3C       # Indicates the transceiver is in a
                                   # transmitting states.
    CST_XCVR_STATE_REQ = 0x3D      # Indicates the transceiver is in a
                                   # requesting state, having transmitted a
                                   # message and is waiting for a response to
                                   # be received.
    CST_XCVR_STATE_RX = 0x3E       # Indicates the transceiver is in a
                                   # receiving state.
    CST_XCVR_STATE_RESP = 0x3F     # Indicates the transceiver is in a
                                   # responding state, where a message is
                                   # being composed and the "response time"
                                   # period is being observed.

