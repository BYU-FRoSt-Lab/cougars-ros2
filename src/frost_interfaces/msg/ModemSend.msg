std_msgs/Header header

# Definitions of Seatrac defined enums and types (ie CID_E, AMSGTYPE_E, NAV_QUERY_T, ...) can be found
# in the Seatrac Developer Guide https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf

# ModemSend is used to transmit an acoustic message to a remote beacon.

uint8 msg_id           # CID_E - Command Identification Code. Indicates the type of command to send to the beacon.
uint8 dest_id          # BID_E. Beacon id of the beacon to send the acoustic transmission to. (1-15 or 0 for all beacons)
uint8 msg_type         # AMSGTYPE_E. Specifies how the remote beacon should respond to the transmission. 
                       # Used with the PING, DAT, and ECHO protocols but not for the NAV protocol. 

uint8 packet_len       # optional packet length for data send commands (0 to 30)
uint8[30] packet_data  # optional packet data array for data send commands

uint8 nav_query_flags  # NAV_QUERY_T. Bit mask indicating which fields the NAV response should return.
                       # Used only with the NAV protocol

