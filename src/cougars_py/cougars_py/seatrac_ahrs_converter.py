
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

        self.modem_subscriber_ = self.create_subscription(
            ModemRec, "modem_rec", self.modem_callback, 10
        )
        self.modem_orientation_pub_ = self.create_publisher(
            PoseWithCovarianceStamped, "modem_orientation", 10
        )
        self.modem_imu_pub_ = self.create_publisher(
            Imu, "modem_imu", 10
        )
    
    def modem_callback(self, msg):
        if msg.msg_id == CID_E.CID_STATUS:
            
            modem_imu = Imu()

            # orientation = PoseWithCovarianceStamped()

            yaw   = 0.1 * msg.attitude_yaw
            pitch = 0.1 * msg.attitude_pitch
            roll  = 0.1 * msg.attitude_roll

            rot1 = R.from_euler('zyx', angles=(yaw, pitch, roll) ,degrees=True)
            #TODO: get rotation with roll as well. It's complicated, so saving for later

            q = rot1.as_quat()

            modem_imu.orientation.x = q[0]
            modem_imu.orientation.y = q[1]
            modem_imu.orientation.z = q[2]
            modem_imu.orientation.w = q[3]

            modem_imu.angular_velocity.x = msg.gyro_x
            modem_imu.angular_velocity.y = msg.gyro_y
            modem_imu.angular_velocity.z = msg.gyro_z

            modem_imu.linear_acceleration.x = msg.acc_x
            modem_imu.linear_acceleration.y = msg.acc_y
            modem_imu.linear_acceleration.z = msg.acc_z

            self.modem_imu_pub_.publish(modem_imu)

            # orientation.pose.covariance = np.array(  # TODO: tune covariance
            #    [0, 0, 0, 0, 0, 0,
            #     0, 0, 0, 0, 0, 0,
            #     0, 0, 0, 0, 0, 0,
            #     0, 0, 0, 1, 0, 0,
            #     0, 0, 0, 0, 1, 0,
            #     0, 0, 0, 0, 0, 1],
            #     dtype=np.float64
            # )
            
            # self.modem_orientation_pub_.publish(orientation)


def main(args=None):
    rclpy.init(args=args)
    seatrac_logger = SeatracAHRSConverter()
    rclpy.spin(seatrac_logger)
    rclpy.shutdown()


if __name__ == '__main__':
    main()