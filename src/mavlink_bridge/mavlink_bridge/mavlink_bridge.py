import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from pymavlink import mavutil

class MAVLinkBridge(Node):
    def __init__(self):
        super().__init__('mavlink_bridge')
        self.publisher = self.create_publisher(Imu, '/mavros/imu', 10)
        self.mavlink_connection = mavutil.mavlink_connection("udp:localhost:14000")
        self.timer = self.create_timer(0.1, self.publish_imu)

    def publish_imu(self):
        msg = self.mavlink_connection.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            imu_msg = Imu()
            imu_msg.orientation.x = msg.roll
            imu_msg.orientation.y = msg.pitch
            imu_msg.orientation.z = msg.yaw
            self.publisher.publish(imu_msg)
            self.get_logger().info("Published IMU data")

def main(args=None):
    rclpy.init(args=args)
    node = MAVLinkBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
