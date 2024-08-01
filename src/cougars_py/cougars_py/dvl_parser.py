import crcmod
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from frost_interfaces.msg import DVLStrings

TEST_WRU = "wru,0,0.070,1.10,-40,-95*9c"


class DVLParser(Node):
    # Creates all of the publishers, subscriptions, services, and clients
    def __init__(self):
        super().__init__('dvl_parser')

        self.subscription_dvl_data = self.create_subscription(DVLStrings,'dvl_data',self.dvl_listener,10)
        self.parsed_dvl_data = {}

        self.publisher_dvl_velocity = self.create_publisher(TwistWithCovarianceStamped, 'dvl_velocity', 10)
        self.publisher_dvl_pose = self.create_publisher(PoseWithCovarianceStamped, 'dvl_pose', 10)
        self.publisher_dvl_depth = self.create_publisher(Float64, 'dvl_depth', 10)
     
    def do_checksum(self, dvl_string):

        crc = crcmod.predefined.mkPredefinedCrcFun("crc-8")
        sentence = bytes(dvl_string,'utf-8')
        data, checksum = sentence.split(b"*")
        if crc(data) == int(checksum, 16):
            return True
        else:
            return False
    
    def parse_wrz(self,wrz):

        if self.do_checksum(wrz):
            data = wrz.split("*")[0].split(",")
            parsed_wrz = {}

            # Report label
            report_label = data[0]
            # Velocity in x direction (m/s)
            parsed_wrz["vx"] = data[1]
            # Velocity in y direction (m/s)
            parsed_wrz["vy"] = data[2]
            # Velocity in z direction (m/s)
            parsed_wrz["vz"] = data[3]
            # If y, the DVL has a lock on the reflecting surface, and the altitude and velocities are valid (y/n)
            parsed_wrz["valid"] = data[4]
            # Measured altitude to the bottom (m)
            parsed_wrz["altitude"] = data[5]
            # Figure of merit, a measure of the accuracy of the velocities (m/s)
            parsed_wrz["fom"] = data[6]
            # Covariance matrix for the velocities. The figure of merit is calculated from this. 9 entries ((m/s)^2) separated by 
            parsed_wrz["covariance"] = data[7]
            # Timestamp of the surface reflection, aka 'center of ping' (Unix timestamp in microseconds)
            parsed_wrz["time_of_validity"] = data[8]
            # Timestamp from immediately before sending of the report over TCP (Unix timestamp in microseconds)
            parsed_wrz["time_of_transmission"] = data[9]
            # Milliseconds since last velocity report (ms)
            parsed_wrz["time"] = data[10]
            # 8 bit status mask. Bit 0 is set to 1 for high temperature and DVL will soon enter thermal shutdown. Remaining bits are reserved for future use.
            parsed_wrz["status"] = data[11]

            # add to the parsed_dvl_data
            self.parsed_dvl_data[report_label] = parsed_wrz

    def parse_wrp(self,wrp):

        if self.do_checksum(wrp):
            data = wrp.split("*")[0].split(",")
            parsed_wrp = {}

            # Report label
            report_label = data[0]
            # Time stamp of report (Unix timestamp in seconds)
            parsed_wrp["time_stamp"] = data[1]
            # Distance in X direction (m)
            parsed_wrp["x"] = data[2]
            # Distance in Y direction (m)
            parsed_wrp["y"] = data[3]
            # Distance in downward direction (m)
            parsed_wrp["z"] = data[4]
            # Standard deviation (Figure of merit) for position (m)
            parsed_wrp["pos_std"] = data[5]
            # Rotation around X axis (degrees)
            parsed_wrp["roll"] = data[6]
            # Rotation around Y axis (degrees)
            parsed_wrp["pitch"] = data[7]
            # Rotation around Z axis, i.e. heading (degrees)
            parsed_wrp["yaw"] = data[8]
            # Reports if there are any issues with the DVL (0 if no errors, 1 otherwise)
            parsed_wrp["status"] = data[9]

            # add to the parsed_dvl_data
            self.parsed_dvl_data[report_label] = parsed_wrp

    def parse_wru(self,wru):

        if self.do_checksum(wru):
            data = wru.split("*")[0].split(",")
            parsed_wru = {}

            # Report label
            report_label = data[0]
            # Transducer number
            parsed_wru["id"] = data[1]
            # Velocity in the direction of the transducer (m/s)
            parsed_wru["velocity"] = data[2]
            # Distance (parallel to the transducer beam, i.e. not the vertical distance) to the reflecting surface from this transducer (m)
            parsed_wru["distance"] = data[3]
            # Received signal strength indicator: strength of the signal received by this transducer (dBm)
            parsed_wru["rssi"] = data[4]
            # Noise spectral density: strength of the background noise received by this transducer (dBm)
            parsed_wru["nsd"] = data[5]

            # add to the parsed_dvl_data
            self.parsed_dvl_data[report_label] = parsed_wru

    def dvl_listener(self, msg):

        # velocity report
        self.parse_wrz(msg.wrz)
        # dead reckoning report
        self.parse_wrp(msg.wrp)
        # transducer report
        self.parse_wru(msg.wru)

        self.velocity_publish() # TwistWithCovarianceStamped
        self.depth_publish() # Float64
        self.state_publish() # PoseWithCovarianceStamped

    def depth_publish(self):
        msg = Float64()
        
        msg.data = self.parsed_dvl_data['wru']['distance']

        self.publisher_dvl_depth.publish(msg)


    def state_publish(self):
        msg = PoseWithCovarianceStamped()

        msg.pose.quaternion.x = self.parsed_dvl_data['wrp']['roll']
        msg.pose.quaternion.y = self.parsed_dvl_data['wrp']['pitch']
        msg.pose.quaternion.z = self.parsed_dvl_data['wrp']['yaw']
        
        msg.pose.point.x = self.parsed_dvl_data['wrp']['x']
        msg.pose.point.y = self.parsed_dvl_data['wrp']['y']
        msg.pose.point.z = self.parsed_dvl_data['wrp']['z']
        
        self.publisher_dvl_pose.publish(msg)

    def velocity_publish(self):
        msg = TwistWithCovarianceStamped()

        msg.twist.twist.linear.x = self.parsed_dvl_data['wrz']['vx']
        msg.twist.twist.linear.y = self.parsed_dvl_data['wrz']['vy']
        msg.twist.twist.linear.z = self.parsed_dvl_data['wrz']['vz']

        self.publisher_dvl_velocity.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    parser = DVLParser()

    rclpy.spin(parser)

    parser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()