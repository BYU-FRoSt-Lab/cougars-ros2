import time
import gpiod

import rclpy
from rclpy.node import Node
from frost_interfaces.msg import DesiredDepth, DesiredHeading, DesiredSpeed, ModemSend
from frost_interfaces.srv import EmergencyStop
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from .seatrac_utils import hello_world_modem_send

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
        self.nav_publisher = self.create_publisher(
            DesiredDepth,
            "desired_depth",
            qos_profile_system_default
        )

        self.nav_publisher = self.create_publisher(
            DesiredHeading,
            "desired_heading",
            qos_profile_system_default
        )

        self.nav_publisher = self.create_publisher(
            DesiredSpeed,
            "desired_speed",
            qos_profile_system_default
        )

        self.modem_publisher = self.create_publisher(
            ModemSend,
            "modem_send",
            qos_profile_sensor_data
        )

        # TODO: remove after publishing once to test modem sending capability
        self.get_logger().info("Sent Hello world to modem")
        self.modem_publisher.publish(hello_world_modem_send())

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

        # TODO: Integrate into MOOS Bridge somehow
        if ENABLE_STROBE:
            if time.time() % 2 < 1:
                self.control_line.set_value(1)
            else:
                self.control_line.set_value(0)

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