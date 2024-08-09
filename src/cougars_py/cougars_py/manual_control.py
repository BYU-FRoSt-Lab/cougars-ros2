import time
import gpiod

import rclpy
from rclpy.node import Node
from frost_interfaces.msg import DesiredDepth, DesiredHeading, DesiredSpeed, ModemSend
from frost_interfaces.srv import EmergencyStop
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

STROBE_PIN = 15
ENABLE_STROBE = True

COMMAND_TIMER_PERIOD = 0.5 # seconds


class ManualControl(Node):
    # Creates all of the publishers, subscriptions, services, and clients
    def __init__(self):
        super().__init__("manual_control")

        # Declare parameters
        # These are set using the local "set_config.py" file when "start.sh" is run
        self.declare_parameter('vehicle_id', 0)

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

        self.modem_publisher = self.create_publisher(
            ModemSend,
            "modem_send",
            qos_profile_sensor_data
        )

        # Create the timers
        self.timer = self.create_timer(
            COMMAND_TIMER_PERIOD,
            self.timer_callback
        )

        # Create the services
        self.srv = self.create_service(
            EmergencyStop,
            "emergency_stop",
            self.emergency_stop_callback
        )

        # Set up the strobe light
        if ENABLE_STROBE:
            self.chip = gpiod.Chip('gpiochip4')
            self.control_line = self.chip.get_line(STROBE_PIN)
            self.control_line.request(consumer="STROBE", type=gpiod.LINE_REQ_DIR_OUT)

        self.counter = 0
        self.stopped = False

    # Runs the state machine and high-level controller, publishes to pid_request
    def timer_callback(self):

        # blink the strobe every second
        if ENABLE_STROBE:
            if time.time() % 2 < 1:
                self.control_line.set_value(1)
            else:
                self.control_line.set_value(0)

        ##########################################################
        # HIGH-LEVEL CONTROLLER CODE STARTS HERE
        # - For a faster update time, adjust PUB_TIMER_PERIOD
        ##########################################################

        depth_msg = DesiredDepth()
        heading_msg = DesiredHeading()
        speed_msg = DesiredSpeed()

        # TODO: Adjust this simple state machine
        if not self.stopped and self.counter < 30:
            depth_msg.desired_depth = 1.0
            heading_msg.desired_heading = 0.0
            speed_msg.desired_speed = 0.0
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