#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class PositionPlotter(Node):
    def __init__(self):
        super().__init__('position_plotter')

        # Initialize variables for storing positions
        self.x_data_old = []
        self.y_data_old = []
        self.x_data_new = []
        self.y_data_new = []

        # Create a subscription to the "smoothed_output" topic
        self.subscription_old = self.create_subscription(
            Odometry,
            'smoothed_output',
            self.odom_callback_old,
            10  # QoS history depth
        )
        self.subscription_new = self.create_subscription(
            Odometry,
            'new_smoothed_output',
            self.odom_callback_new,
            10  # QoS history depth
        )
        self.get_logger().info("Subscribed to 'smoothed_output' topic")

        # Initialize the plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Robot Position")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        plt.ion()  # Interactive mode for live updates
        plt.show()

    def odom_callback_old(self, odom_msg):
        self.get_logger().info("Got old data")
        # Extract x and y positions from the Odometry message
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        # Append the positions to the data lists
        self.x_data_old.append(x)
        self.y_data_old.append(y)

        # Update the plot
        self.update_plot()

    def odom_callback_new(self, odom_msg):
        self.get_logger().info("Got new data")

        # Extract x and y positions from the Odometry message
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        # Append the positions to the data lists
        self.x_data_new.append(x)
        self.y_data_new.append(y)

        # Update the plot
        self.update_plot()

    def update_plot(self):
        self.ax.clear()
        self.ax.plot(self.x_data_old, self.y_data_old, '-o', label="Trajectory Old", color='black')
        self.ax.plot(self.x_data_new, self.y_data_new, '-*', label="Trajectory New")
        self.ax.legend()
        self.ax.grid()
        self.ax.set_title("Robot Position")
        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        plt.pause(0.01)

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the node
    plotter = PositionPlotter()

    try:
        # Spin the node to keep it active
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
