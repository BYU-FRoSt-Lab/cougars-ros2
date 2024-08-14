
import rcopy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from frost_interfaces.msg import ModemRec
from .seatrac_enums import CID_E
from scipy.spatial.transform import rotation



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
            #TODO: add code to calculate and extract orientation
            self.modem_orientation_pub_.publish(orientation)
            
