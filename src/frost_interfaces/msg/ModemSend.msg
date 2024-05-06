std_msgs/Header header

# see seatrac developer manual for specific values
# https://www.seascapesubsea.com/downloads/Blueprint-Subsea-SeaTrac-Developer-Guide.pdf

uint8 msg_id             # CID_E command identification code (type of message sent)
uint8 dest_id            # BID_ID beacon id destination (1-15 or 0 for all beacons)
uint8 msg_type           # AMSGTYPE_E further specifies the type of messages
uint8 packet_len         # optional packet length for data send commands (0 to 31)
uint8[] packet_data      # optional packet data array for data send commands


