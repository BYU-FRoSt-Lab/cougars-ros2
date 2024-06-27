std_msgs/Header header

uint8 dest_id
uint8 src_id

uint8 msg_id            # CID_E command identification code (type of message sent)
uint8 packet_len        # packet length for data send messages
uint8[31] packet_data   # packet data for data send messages

bool local_flag         # True if the message was addressed to this beacon. False if it was "sniffed" by the beacon.
bool position_enhanced  # True if depth was sent and used to compute values. False if depth info was not sent.
bool position_flt_error # True if the position filter detected a discrepancy between this position and the last position

### the following values relate to the current receiving beacon when the message was computed ###
int16 attitude_yaw
int16 attitude_pitch 
int16 attitude_roll
int16 depth_local   # local (receiving beacon) depth
uint16 vos          # velocity of sound
uint16 rssi         # recieved signal strength indicator

### range fields ###
bool range_valid     # range fields are only valid if this is true
uint32 range_count   # number of 16kHz timer intervals counted between req and resp
int32 range_time     # divide by 10 mil for seconds 
uint16 range_dist    # divide by 10 for range in meters

### USBL Fields ###
bool usbl_valid      # usbl fields only valid if true
uint8 usbl_channels  # number of usbl channels used to compute signal (usually 3/4)
int16[4] usbl_rssi
int16 usbl_azimuth   # azimuth of sending beacon relative to receiving beacon
int16 usbl_elevation
int16 usbl_fit_error

### position fields ###
bool position_valid
int16 position_easting
int16 position_northing
int16 position_depth

