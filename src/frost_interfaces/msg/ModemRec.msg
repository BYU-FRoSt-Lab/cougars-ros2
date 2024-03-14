std_msgs/Header header

uint8 msgId         # CID_E command identification code (type of message sent)
uint8 packetLen     # packet length for data send messages
uint8[31] packetData    # packet data for data send messages

### the following values relate to the current beacon when the message was computed ###
int16 attitudeYaw
int16 attitudePitch 
int16 attitudeRoll
int16 depthLocal    # local (receiving beacon) depth
uint16 VOS          # velocity of sound
uint16 RSSI         # recieved signal strength indicator

### range fields ###
bool rangeValid     # range fields are only valid if this is true
uint32 rangeCount   # number of 16kHz timer intervals counted between req and resp
int32 rangeTime     # divide by 10 mil for seconds 
uint16 rangeDist    # divide by 10 for range in meters

### USBL Fields ###
bool usblValid      # usbl fields only valid if true
uint8 usblChannels  # number of usbl channels used to compute signal (usually 3/4)
int16[4] usblRSSI
int16 usblAzimuth   # azimuth of sending beacon relative to receiving beacon
int16 usblElevation
int16 usblFitError

### position fields ###
bool positionValid
int16 positionEasting
int16 positionNorthing
int16 positionDepth

