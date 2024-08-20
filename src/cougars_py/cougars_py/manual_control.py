import time
import gpiod

import rclpy
from rclpy.node import Node
from frost_interfaces.msg import DesiredDepth, DesiredHeading, DesiredSpeed
from frost_interfaces.srv import EmergencyStop
from rclpy.qos import qos_profile_system_default


class ManualControl(Node):
    # Creates all of the publishers, subscriptions, services, and clients
    def __init__(self):
        super().__init__("manual_control")

        # Declare parameters
        self.declare_parameter('vehicle_id', 0)
        self.declare_parameter('command_timer_period', 0.5) # in seconds
        self.declare_parameter('state_1_count', 0) # in intervals based on command_timer_period
        self.declare_parameter('state_1_depth', 0.0)
        self.declare_parameter('state_1_heading', 0.0)
        self.declare_parameter('state_1_speed', 0.0)
        self.declare_parameter('state_2_count', 0) # in intervals based on command_timer_period
        self.declare_parameter('state_2_depth', 0.0)
        self.declare_parameter('state_2_heading', 0.0)
        self.declare_parameter('state_2_speed', 0.0)
        self.declare_parameter('state_3_count', 0) # in intervals based on command_timer_period
        self.declare_parameter('state_3_depth', 0.0)
        self.declare_parameter('state_3_heading', 0.0)
        self.declare_parameter('state_3_speed', 0.0)

        # Create the publishers
        self.depth_publisher = self.create_publisher(
            DesiredDepth,
            "desired_depth",
            qos_profile_system_default
        )

        self.heading_publisher = self.create_publisher(
            DesiredHeading,
            "desired_heading",
            qos_profile_system_default
        )

        self.speed_publisher = self.create_publisher(
            DesiredSpeed,
            "desired_speed",
            qos_profile_system_default
        )

        # Create the timers
        self.timer = self.create_timer(
            self.get_parameter("command_timer_period").get_parameter_value().double_value,
            self.timer_callback
        )

        # Create the services
        self.srv = self.create_service(
            EmergencyStop,
            "emergency_stop",
            self.emergency_stop_callback
        )

        self.counter = 0
        self.stopped = False

    # Runs the state machine and high-level controller, publishes to pid_request
    def timer_callback(self):

        ##########################################################
        # HIGH-LEVEL CONTROLLER CODE STARTS HERE
        # - For a faster update time, adjust PUB_TIMER_PERIOD
        ##########################################################

        depth_msg = DesiredDepth()
        heading_msg = DesiredHeading()
        speed_msg = DesiredSpeed()

        # TODO: Adjust this simple state machine
        if not self.stopped and self.counter < self.get_parameter("state_1_count").get_parameter_value().integer_value:
            depth_msg.desired_depth = self.get_parameter("state_1_depth").get_parameter_value().double_value
            heading_msg.desired_heading = self.get_parameter("state_1_heading").get_parameter_value().double_value
            speed_msg.desired_speed = self.get_parameter("state_1_speed").get_parameter_value().double_value
        elif not self.stopped and self.counter < self.get_parameter("state_1_count").get_parameter_value().integer_value + self.get_parameter("state_2_count").get_parameter_value().integer_value:
            depth_msg.desired_depth = self.get_parameter("state_2_depth").get_parameter_value().double_value
            heading_msg.desired_heading = self.get_parameter("state_2_heading").get_parameter_value().double_value
            speed_msg.desired_speed = self.get_parameter("state_2_speed").get_parameter_value().double_value
        elif not self.stopped and self.counter < self.get_parameter("state_1_count").get_parameter_value().integer_value + self.get_parameter("state_2_count").get_parameter_value().integer_value + self.get_parameter("state_3_count").get_parameter_value().integer_value:
            depth_msg.desired_depth = self.get_parameter("state_3_depth").get_parameter_value().double_value
            heading_msg.desired_heading = self.get_parameter("state_3_heading").get_parameter_value().double_value
            speed_msg.desired_speed = self.get_parameter("state_3_speed").get_parameter_value().double_value
        else:
            depth_msg.desired_depth = 0.0
            heading_msg.desired_heading = 0.0
            speed_msg.desired_speed = 0.0

        # Publish the messages
        self.depth_publisher.publish(depth_msg)
        self.heading_publisher.publish(heading_msg)
        self.speed_publisher.publish(speed_msg)

        self.get_logger().info("Depth: %f, Heading: %f, Speed: %f" % (
            depth_msg.desired_depth,
            heading_msg.desired_heading,
            speed_msg.desired_speed,
        ))

        self.counter += 1

        ##########################################################
        # HIGH-LEVEL CONTROLLER CODE ENDS HERE
        ##########################################################

    # Logs when EmergencyStop is requested
    def emergency_stop_callback(self, request, response):
        self.get_logger().info("EMERGENCY STOP EXECUTED")
        self.get_logger().info(request.error)
        self.stopped = True
        response.stopped = True
        return response


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    manual_control = ManualControl()
    executor.add_node(manual_control)

    executor.spin()

    executor.shutdown()
    manual_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
