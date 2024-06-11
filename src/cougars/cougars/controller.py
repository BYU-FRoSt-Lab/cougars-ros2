import rclpy
import gpiod
import time
from rclpy.node import Node
from enum import Enum
from frost_interfaces.msg import PID, Echo, GPS, ModemSend
from frost_interfaces.srv import EmergencyStop
from .seatrac_utils import hello_world_modem_send

PID_PUB_TIMER_PERIOD = 1 # seconds
LED_PIN = 15


class States(Enum):
    RUN = 1
    STOP = 2


class Controller(Node):
    # Creates all of the publishers, subscriptions, services, and clients
    def __init__(self):
        super().__init__("controller")

        # Create the callback groups
        # main_callback_group - functions outside of the timer callback loop
        # aux_callback_group - functions inside of the timer callback loop
        self.main_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.aux_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # Create the publishers
        self.nav_publisher = self.create_publisher(
            PID,
            "pid_request",
            10,
            callback_group=self.main_callback_group
        )

        self.modem_publisher = self.create_publisher(
            ModemSend,
            "modem_send",
            10,
            callback_group=self.aux_callback_group
        )
        # TODO: remove after publishing once to test modem sending capability
        self.get_logger().info("Sent Hello world to modem")
        self.modem_publisher.publish(hello_world_modem_send())
        
        # Create the timers
        self.timer = self.create_timer(
            PID_PUB_TIMER_PERIOD,
            self.timer_callback,
            callback_group=self.main_callback_group
        )

        # Create the services
        self.srv = self.create_service(
            EmergencyStop,
            "emergency_stop",
            self.emergency_stop_callback,
            callback_group=self.main_callback_group,
        )

        # Create the GPIO chip and line
        self.chip = gpiod.Chip('gpiochip4')
        self.led_line = self.chip.get_line(LED_PIN)
        self.led_line.request(consumer="LED", type=gpiod.LINE_REQ_DIR_OUT)

        # Set initial variables
        self.state = States.RUN
        self.counter = 0

    # Sets the state machine to STOP when EmergencyStop is requested
    def emergency_stop_callback(self, request, response):
        self.get_logger().info("EMERGENCY STOP EXECUTED")
        self.get_logger().info(request.error)
        self.state = States.STOP
        response.stopped = True
        return response

    # Runs the state machine and high-level controller, publishes to pid_request
    def timer_callback(self):
        pid_msg = PID()

        # Blink the LED
        if time.time() % 2 < 1:
            self.led_line.set_value(0)
        else:
            self.led_line.set_value(1)

        if self.state == States.RUN:

            ##########################################################
            # HIGH-LEVEL CONTROLLER CODE STARTS HERE
            # - For a faster update time, adjust PID_PUB_TIMER_PERIOD
            ##########################################################

            # TODO: Adjust this simple state machine
            if self.counter < 30:
                pid_msg.velocity = 0.0
                pid_msg.yaw = 90.0 # in degrees
                pid_msg.pitch = 0.0 # maybe not needed?
                pid_msg.roll = 0.0 # maybe not needed?
                pid_msg.depth = 0.0
                pid_msg.stop = False
            else:
                self.state = States.STOP

            self.counter += 1

            self.get_logger().info("PUBLISHING TO PID_REQUEST")

            ##########################################################
            # HIGH-LEVEL CONTROLLER CODE ENDS HERE
            ##########################################################

        elif self.state == States.STOP:
            pid_msg.stop = True

        # Publish the PID message
        pid_msg.header.stamp = Node.get_clock(self).now().to_msg()
        self.get_logger().info("Velocity (%d), Heading (%d, %d, %d), Depth (%d)" % (
            pid_msg.velocity,
            pid_msg.yaw,
            pid_msg.pitch,
            pid_msg.roll,
            pid_msg.depth,
        ))
        self.nav_publisher.publish(pid_msg)


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    controller = Controller()
    executor.add_node(controller)

    executor.spin()

    executor.shutdown()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()