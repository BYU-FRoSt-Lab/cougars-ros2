#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Header, Bool
from frost_interfaces.msg import SystemControl  # Change to your actual package name

import time


class SystemStatusPublisher(Node):
    def __init__(self):
        super().__init__('system_status_publisher')

        # Set reliable and transient local QoS profile
        qos_reliable_profile = QoSProfile(depth=5)
        qos_reliable_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_reliable_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher_ = self.create_publisher(SystemControl, 'system/status', qos_reliable_profile)
        self.get_logger().info("SystemStatusPublisher node started. Preparing message...")

        self.publish_user_input()

    def publish_user_input(self):
        msg = SystemControl()

        # Fill header with timestamp
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'system_status_input'


        # TODO let the user put in the parameters in the command line
        # Prompt user for input
        try:
            start_input = input("Start the node? (y/n): ").strip().lower()
            rosbag_flag_input = input("Record rosbag? (y/n): ").strip().lower()
            rosbag_prefix_input = input("Enter rosbag prefix (string): ").strip()

            msg.start = Bool(data=start_input == "y")
            msg.rosbag_flag = Bool(data=rosbag_flag_input == "y")
            msg.rosbag_prefix = rosbag_prefix_input

            # Publish message
            self.publisher_.publish(msg)
            self.get_logger().info("Published SystemControl message.")
            self.get_logger().info(f"Start: {msg.start.data}, Rosbag Flag: {msg.rosbag_flag.data}, Prefix: {msg.rosbag_prefix}")

        except Exception as e:
            self.get_logger().error(f"Error getting user input: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SystemStatusPublisher()

    # Allow time for reliable delivery
    rclpy.spin_once(node, timeout_sec=2.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
