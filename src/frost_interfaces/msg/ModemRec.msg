std_msgs/Header header

# Definitions of Seatrac defined enums and types (ie CID_E, BID_E, CST_E, ...) can be found
# in the Seatrac Developer Guide https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf

uint8 msg_id            # CIDE - Command Identification Code. Indicates the type of report recieved from the beacon.
                        # Different msg types include different information, and therefore populate different fields.
                        # Not all CID_E msg types are captured or supported in this ros message.

bool  from_acoustic_tx  # True if this report was generated from an acoustic transmission successfully received from a remote beacon
                        # False if this report is a Status message or response to a command from the client program


### Command Status Code Fields ###
        # Command Status Codes tell you if a command executed sucessfully, and if not, what error the command encountered
        # Reports that include a command status code usually do not include any other fields besides msg_id and local_beacon_id
bool  includes_command_status_code
uint8 command_status_code
uint8 target_id   # BID_E. For error reports from acoustic transmissions, the target_id is the id of the remote beacon.


### Acoustic Fix OR Status Fields ###
        # Fields included in both Acoustic Fix reports and Status Reports
bool     includes_local_attitude
int16    attitude_yaw    # Divide by 10 for local yaw in degrees
int16    attitude_pitch  # Divide by 10 for local pitch in degrees
int16    attitude_roll   # Divide by 10 for local roll in degrees

bool     includes_local_depth_and_vos
int16    depth_local  # Divide by 10 for local depth im meters
uint16   vos          # Divide by 10 for velocity of sound in meters per second.


### Status Fields ###
        # This report is a Status message if CID_E msg_id = CID_STATUS = 0x10
        # Status messages can be generated at a user defined interval between 1 and 25 Hz
bool   includes_status_timestamp
uint64 timestamp  # time since beacon was last powered up in milliseconds

bool   includes_status_env_fields
uint16 supply_voltage  # Beacon's supply voltage. Divide by 1000 for value in Volts.
int16  env_temp        # Temperature of air/water in contact with pressure sensor. Divide by 10 for value in Celsius.
int32  env_pressure    # External air/water pressure in milli-bars, with tolerance of +\-1 Bar.


### Acoustic Fix (ACOFIX_T) Fields ###
        # An Acoustic Fix structure is generated for every successfully received acoustic transmission.
bool  local_flag          # True if the transmission was addressed to this beacon. False if it was "sniffed" by the beacon.
bool  position_enhanced   # True if depth was sent in the acoustic transmission. False if depth was not transmitted.
bool  position_flt_error  # True if the position filter detected a discrepancy between this position and the last position.

uint8 dest_id  # BID_E. The id of the beacon the acoustic transmission was addressed to. 
               #        Usually the same as the beacon that genrated this report.
uint8 src_id   # BID_E. The id of the remote beacon that sent the acoustic transmission.

uint16 rssi  # rssi for main transceiver. Divide by 10 for rssi in decibels.

bool     includes_range  # Indicates whether or not range information could be included in this report.
uint32   range_count     # Number of 16kHz timer intervals counted between req and resp.
int32    range_time      # Time between req and resp. Divide by 10 for time in microseconds.
uint16   range_dist      # Divide by 10 for range in meters.

bool     includes_usbl   # Indicates whether or not usbl information could be included in this report.
uint8    usbl_channels   # number of usbl channels used to compute signal (usually 3 or 4)
int16[4] usbl_rssi       # rssi for each usbl reciever. Divide by 10 for rssi in decibels.
int16    usbl_azimuth    # Azimuth of remote beacon relative to local beacon. Divide by 10 for azimuth in degrees.
int16    usbl_elevation  # Elevation of remote beacon relative to local beacon. Divide by 10 for elevation in degrees.
int16    usbl_fit_error  # Dimensionless value indicating level of confidence in usbl azimuth and elevation
                         #     where 0 is high confidence and 2 or greater is low confidence.

bool     includes_position  # Indicates whether or not position information could be included in this report.
int16    position_easting   # Easting component of relative position to remote beacon. Divide by 10 for value in meters.
int16    position_northing  # Northing component of relative position to remote beacon. Divide by 10 for value in meters.
int16    position_depth     # Vertical depth compoenent of remote beacon from the surface. Divide by 10 for value in meters.
                            #     If position_enhanced is true, this value is the transmitted depth sensor value from the remote beacon.
                            #     Otherwise, this value is computed from the usbl information.


### Acoustic Transmission Data Fields ###
uint8     packet_len    # Packet length. If packet length is 0, the message conains no packet data.
uint8[30] packet_data   # Data packet received from remote beacon. Can be no longer than 30 bytes.


### NAV fields ###
        # The NAV protocol is used to query information from remote beacons.
        # If the recieved acoustic transmission was a response to a NAV query transmission,
        # it may include the following information.
bool   includes_remote_depth
int32  remote_depth  # Divide by 10 for remote depth in meters.

bool   includes_remote_supply
uint16 remote_supply # Divide by 1000 for remote supply voltage in Volts.

bool   includes_remote_temp
int16  remote_temp   # Divide by 10 for remote temperature in Celsius.

bool   includes_remote_attitude
int16  remote_yaw    # Divide by 10 for remote yaw in degrees.
int16  remote_pitch  # Divide by 10 for remote pitch in degrees.
int16  remote_roll   # Divide by 10 for remote roll in degrees.


