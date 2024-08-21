import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from scipy.spatial.transform import Rotation as R
import numpy as np


class FactorGraphNode(Node):

    def __init__(self):
        super().__init__('factor_graph_node')

        # Initialize class variables
        self.orientation_matrix = np.eye(3)
        self.orientation_covariance = np.zeros((3, 3))
        self.position = np.zeros(3)
        self.position_covariance = np.zeros((3, 3))
        self.dvl_position = np.zeros(3)
        self.dvl_position_covariance = np.zeros((3, 3))
        self.init_state = {}

        # Subscribers
        self.create_subscription(Imu, '/modem_imu', self.imu_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/depth_data', self.depth_callback, 10)
        self.create_subscription(Odometry, '/gps_odom', self.gps_callback, 10)
        self.create_subscription(Odometry, '/dvl_dead_reckoning', self.dvl_callback, 10)
        self.create_subscription(Empty, '/init', self.init_callback, 10)

        # Publisher
        self.vehicle_status_pub = self.create_publisher(Odometry, '/vehicle_status', 10)

    def imu_callback(self, msg: Imu):
        # Convert quaternion to rotation matrix
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r = R.from_quat(quat)
        self.orientation_matrix = r.as_matrix()

        # Get the orientation covariance
        self.orientation_covariance = np.array(msg.orientation_covariance).reshape(3, 3)

    def depth_callback(self, msg: PoseWithCovarianceStamped):
        # Set the z position and covariance
        self.position[2] = msg.pose.pose.position.z
        self.position_covariance[2,2] = msg.pose.covariance[14]  # Covariance for Z

    def gps_callback(self, msg: Odometry):
        # Get the x, y position and position covariance
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position_covariance = np.array(msg.pose.covariance).reshape(3, 3)

    def dvl_callback(self, msg: Odometry):
        # Get the x, y, z position
        self.dvl_position[0] = msg.pose.pose.position.x
        self.dvl_position[1] = msg.pose.pose.position.y
        self.dvl_position[2] = msg.pose.pose.position.z

        # Convert quaternion to rotation matrix
        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        r = R.from_quat(quat)
        self.dvl_orientation_matrix = r.as_matrix()

    def init_callback(self, msg: Empty):
        # Store current state as the initial state
        self.init_state = {
            'position': self.position.copy(),
            'orientation_matrix': self.orientation_matrix,
            'dvl_position': self.dvl_position.copy()
        }
        self.get_logger().info("Initial state has been set.")

    def publish_vehicle_status(self):
        odom_msg = Odometry()

        # Set the orientation in the message
        r = R.from_matrix(self.orientation_matrix)
        quat = r.as_quat()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set the position in the message
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.z_position

        # Set the covariance
        odom_msg.pose.covariance = [
            self.position_covariance[0, 0], 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.position_covariance[1, 1], 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, self.z_covariance, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, self.orientation_covariance[0, 0], 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, self.orientation_covariance[1, 1], 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.orientation_covariance[2, 2]
        ]

        # Publish the vehicle status
        self.vehicle_status_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FactorGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
