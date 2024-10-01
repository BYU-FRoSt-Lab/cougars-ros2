
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from frost_interfaces.msg import ModemRec
from .seatrac_enums import CID_E
import numpy as np

from scipy.spatial.transform import Rotation as R


class SeatracAHRSConverter(Node):

    def __init__(self):
        super().__init__('seatrac_ahrs_converter')

        # https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?
        self.declare_parameter('magnetic_declination', 10.7) # 10.70° E for Utah Lake

        self.modem_subscriber_ = self.create_subscription(
            ModemRec, "modem_rec", self.modem_callback, 10
        )
        self.modem_imu_pub_ = self.create_publisher(
            Imu, "modem_imu", 10
        )

    def modem_callback(self, msg):
        if msg.msg_id == CID_E.CID_STATUS:

            modem_imu = Imu()
            modem_imu.header.stamp = msg.header.stamp

            yaw   = 0.1 * msg.attitude_yaw - self.get_parameter('magnetic_declination').get_parameter_value().double_value
            # since the declination is East, we subtract it from the yaw
            pitch = 0.1 * msg.attitude_pitch
            roll  = 0.1 * msg.attitude_roll

            R_xyz_ned = R.from_euler('zyx', angles=(yaw, pitch, roll) ,degrees=True)

            # convert from North East Down (ned) coordinates to East North Up (enu) coordinates
            R_ned_enu = R.from_matrix([
                [0,-1,0],
                [-1,0,0],
                [0,0,-1]
            ])
            R_xyz_enu = R_xyz_ned * R_ned_enu

            q = R_xyz_enu.as_quat()

            # self.get_logger().info(str(np.round(q*10)))

            modem_imu.orientation.x = q[0]
            modem_imu.orientation.y = q[1]
            modem_imu.orientation.z = q[2]
            modem_imu.orientation.w = q[3]
            modem_imu.orientation_covariance = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ]

            # modem_imu.angular_velocity.x = msg.gyro_x
            # modem_imu.angular_velocity.y = msg.gyro_y
            # modem_imu.angular_velocity.z = msg.gyro_z
            # modem_imu.angular_velocity_covariance = [
            #     1.0, 0.0, 0.0,
            #     0.0, 1.0, 0.0,
            #     0.0, 0.0, 1.0
            # ]

            # modem_imu.linear_acceleration.x = msg.acc_x
            # modem_imu.linear_acceleration.y = msg.acc_y
            # modem_imu.linear_acceleration.z = msg.acc_z
            # modem_imu.linear_acceleration_covariance = [
            #     1.0, 0.0, 0.0,
            #     0.0, 1.0, 0.0,
            #     0.0, 0.0, 1.0
            # ]

            self.modem_imu_pub_.publish(modem_imu)


def main(args=None):
    rclpy.init(args=args)
    seatrac_logger = SeatracAHRSConverter()
    rclpy.spin(seatrac_logger)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 