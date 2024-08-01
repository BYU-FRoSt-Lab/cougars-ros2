import time
import gpiod

import rclpy
from rclpy.node import Node
from frost_interfaces.msg import DesiredDepth, DesiredHeading, DesiredSpeed
from frost_interfaces.srv import EmergencyStop
from rclpy.qos import qos_profile_system_default

STROBE_PIN = 15
ENABLE_STROBE = True


class MOOSBridge(Node):
    # Creates all of the publishers, subscriptions, services, and clients
    def __init__(self):
        super().__init__("moos_bridge")

        # Declare parameters
        # These are set using the local "set_config.sh" file when "start.sh" is run
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

        # Create the services
        self.srv = self.create_service(
            EmergencyStop,
            "emergency_stop",
            self.emergency_stop_callback
        )

    # Logs when EmergencyStop is requested
    # TODO: Integrate into MOOS Bridge somehow
    def emergency_stop_callback(self, request, response):
        self.get_logger().info("EMERGENCY STOP EXECUTED")
        self.get_logger().info(request.error)
        response.stopped = True
        return response


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    moos_bridge = MOOSBridge()
    executor.add_node(moos_bridge)

    executor.spin()

    executor.shutdown()
    moos_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()