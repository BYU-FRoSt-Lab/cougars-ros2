
from frost_interfaces.msg import ModemRec
from frost_interfaces.msg import ModemSend


 #========================================================================#
 #      This file contains definitions of variables used to send          #
 #      and interpret ros messages from the seatrac modem                 #
 #========================================================================#


 # all definitions included in this file are defined in this manual
 # https://www.seascapesubsea.com/downloads/Blueprint-Subsea-SeaTrac-Developer-Guide.pdf


 # BID_E : Beacon Identification Code
 #
 # Beacon Identification (BID) Codes are used to identify a specific beacon
 # that should receive acoustic messages or identify which beacon was the
 # source (sender) of a message. Valid values are in the range from 0 to 15 and
 # are typically send and stored as a UINT8.
class BID_E :
    BEACON_ALL = 0x0   # When used as an address for sending acoustic
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
 # only a subset of CID codes included here. Full list can be found in the 
 # seatrac manual linked above 
 #
class CID_E :
     # PING Protocol Messages
    CID_PING_SEND = 0x40  # Command sent to transmit a PING message.
    CID_PING_REQ = 0x41   # Message sent when a PING request is received.
    CID_PING_RESP = 0x42  # Message sent when a PING response is received or
                           # timeout occurs with the echo response data.
    CID_PING_ERROR = 0x43 # Message sent when a PING response error/timeout
                           # occurs.

    # ECHO Protocol Messages
    CID_ECHO_SEND = 0x48  # Command sent to transmit an ECHO message.
    CID_ECHO_REQ = 0x49   # Message sent when an ECHO request is received.
    CID_ECHO_RESP = 0x4A  # Message sent when an ECHO response is received
                           # or timeout occurs with the echo response data.
    CID_ECHO_ERROR = 0x4B # Message sent when an ECHO response error/timeout
                           # occurs.

    # DAT Protocol Messages
    CID_DAT_SEND = 0x60         # Message sent to transmit a datagram to
                                 # another beacon
    CID_DAT_RECEIVE = 0x61      # Message generated when a beacon receives a
                                 # datagram.
    CID_DAT_ERROR = 0x63        # Message generated when a beacon response
                                 # error/timeout occurs for ACKs.
    CID_DAT_QUEUE_SET = 0x64    # Message sent to set the contents of the
                                 # packet data queue.
    CID_DAT_QUEUE_CLR = 0x65    # Message sent to clear the contents of the
                                 # packet data queue.
    CID_DAT_QUEUE_STATUS = 0x66 # Message sent to obtain the current status
                                 # of the packet data queue.



 # AMSGTYPE_E : Acoustic Message Type.
 #
 # The AMSGTYPE_E classeration is used to specify a type of acoustic message
 # and determines how the message is processed and which responses are
 # generated from beacons.
class AMSGTYPE_E :
    MSG_OWAY = 0x0    # Indicates an acoustic message is sent One-Way and
                       # does not require a response. One-Way messages may
                       # also be broadcast to all beacons if required.  No
                       # USBL information is sent.
    MSG_OWAYU = 0x1   # Indicates an acoustic message is sent One-Way and
                       # does not require a response. One-Way messages may
                       # also be broadcast to all beacons if required.
                       # Additionally the message is sent with USBL acoustic
                       # information allowing an incoming bearing to be
                       # determined by USBL receivers although range cannot
                       # be provided.
    MSG_REQ = 0x2     # Indicates an acoustic message is sent as a Request
                       # type.  This requires the receiver to generate and
                       # return a Response (MSG_RESP) message.  No USBL
                       # information is requested.
    MSG_RESP = 0x3    # Indicate an acoustic message is sent as a Response to
                       # a previous Request message (MSG_REQ).  No USBL
                       # information is returned.
    MSG_REQU = 0x4    # Indicates an acoustic message is sent as a Request
                       # type.  This requires the receiver to generate and
                       # return a Response (MSG_RESP) message.  Additionally
                       # the Response message should be returned with USBL
                       # acoustic information allowing a position fix to be
                       # computed by USBL receivers through the range and
                       # incoming signal angle.
    MSG_RESPU = 0x5   # Indicate an acoustic message is sent as a Response to
                       # a previous Request message (MSG_REQ).  Additionally
                       # the message is sent with USBL acoustic information
                       # allowing the position of the sender to be determined
                       # through the range and incoming signal angle.
    MSG_REQX = 0x6    # Indicates an acoustic message is sent as a Request
                       # type.  This requires the receiver to generate and
                       # return a Response (MSG_RESP) message.  Additionally
                       # the Response message should be returned with extended
                       # Depth and USBL acoustic information allowing a more
                       # accurate position fix to be computed by USBL
                       # receivers through the range remote depth and
                       # incoming signal angle.
    MSG_RESPX = 0x7   # Indicate an acoustic message is sent as a Response to
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


def hello_world_modem_send():
    message = "hello world"
    modemSend = ModemSend()
    modemSend.msg_id      = CID_E.CID_DAT_SEND
    modemSend.dest_id     = BID_E.BEACON_ALL
    modemSend.msg_type    = AMSGTYPE_E.MSG_OWAY #use MSG_OWAYU to include usbl with message
    modemSend.packet_len  = len(message)
    modemSend.packet_data = [ord(c) for c in message]
    return modemSend

