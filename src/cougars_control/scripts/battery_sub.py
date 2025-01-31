#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from frost_interfaces.srv import EmergencyStop
from frost_interfaces.msg import BatteryStatus
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class BatterySubscriber(Node):
    '''
    :author: Nelson Durrant
    :date: September 2024

    A simple ROS2 node that subscribes to the battery/data topic and listens for the battery voltage.
    If the voltage is below a critical threshold, it sends a request to the emergency_stop service to stop the robot.

    Subscribes:
        - battery/data (frost_interfaces/msg/BatteryStatus)

    Clients:
        - emergency_stop (frost_interfaces/srv/EmergencyStop)
    '''
    def __init__(self):
        '''
        Creates a new BatterySubscriber node.
        '''
        super().__init__("battery_subscriber")

        self.declare_parameter('critical_voltage', 14.0)
        '''
        :param critical_voltage: The critical voltage threshold below which the robot should stop. The default value is 14.0.
        '''

        self.subscription = self.create_subscription(
            BatteryStatus, "battery/data", self.listener_callback, qos_profile_sensor_data
        )
        '''
        Subscription to the "battery/data" topic with the message type BatteryStatus.
        '''
        self.subscription  # prevent unused variable warning
        
        self.cli = self.create_client(SetParameters, 'emergency_protocols/set_parameters')  

        while not self.cli.wait_for_service(timeout_sec=1):
            self.get_logger().warn("Setting leak parameter service not available, waiting...")
        self.req = SetParameters.Request()

   

    def listener_callback(self, msg):
        '''
        Callback function for the battery/data subscription.
        If the voltage is below the critical threshold, this method changes an emergency_protocol parameter to stop the robot.

        :param msg: The BatteryStatus message received from the battery/data topic.
        '''
        if msg.voltage < self.get_parameter("critical_voltage").get_parameter_value().double_value:
            low_batt_param = Parameter()
            low_batt_param.name = 'low_battery_detected'
            low_batt_param.value.type = ParameterType.PARAMETER_BOOL
            low_batt_param.value.bool_value = True
            self.req.parameters.append(low_batt_param)
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)

            self.get_logger().info("%s" % (self.future.result()))
            if self.future.result() == False:
                self.get_logger().error("Failed to notify of low battery, emergency low_batt_param not changed")

            


def main(args=None):
    rclpy.init(args=args)
    battery_subscriber = BatterySubscriber()
    rclpy.spin(battery_subscriber)
    battery_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()