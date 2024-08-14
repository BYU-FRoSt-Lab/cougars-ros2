
import rcopy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
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
    
    def modem_callback(self, msg):
        if msg.msg_id == CID_E.CID_STATUS:
            orientation = PoseWithCovarianceStamped()

            yaw   = 0.1 * msg.attitude_yaw
            pitch = 0.1 * msg.attitude_pitch
            roll  = 0.1 * msg.attitude_roll

            rot1 = R.from_euler('xy', angles=(yaw, pitch) ,degrees=True)
            #TODO: get rotation with roll as well. It's complicated, so saving for later

            q = rot1.as_quat()

            orientation.pose.pose.orientation.x = q[0]
            orientation.pose.pose.orientation.y = q[1]
            orientation.pose.pose.orientation.z = q[2]
            orientation.pose.pose.orientation.w = q[3]

            orientation.pose.covariance = np.array(  # TODO: tune covariance
               [0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1],
                dtype=np.float64
            )

            #TODO: solve for covariance

            self.modem_orientation_pub_.publish(orientation)
            
