#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from frost_interfaces.srv import EmergencyStop
from frost_interfaces.msg import LeakStatus
from rclpy.qos import qos_profile_sensor_data


class LeakSubscriber(Node):
    '''
    :author: Nelson Durrant
    :date: September 2024

    A simple ROS2 node that subscribes to the leak/data topic and listens for the leak status.
    If a leak is detected, it sends a request to the emergency_stop service to stop the robot.
    
    Subscribes:
        - leak/data (frost_interfaces/msg/LeakStatus)
    Clients:
        - emergency_stop (frost_interfaces/srv/EmergencyStop)
    '''
    def __init__(self):
        '''
        Creates a new LeakSubscriber node.
        '''
        super().__init__("leak_subscriber")

        self.subscription = self.create_subscription(
            LeakStatus, "leak/data", self.listener_callback, qos_profile_sensor_data
        )
        '''
        Subscription to the "leak/data" topic with the message type LeakStatus.
        '''
        self.subscription  # prevent unused variable warning
        
        self.cli = self.create_client(EmergencyStop, "emergency_stop")
        '''
        Client for the "emergency_stop" service with the service type EmergencyStop.
        '''
        while not self.cli.wait_for_service(timeout_sec=1):
            self.get_logger().info("EmergencyStop service not available, waiting...")
        self.req = EmergencyStop.Request()

    def send_request(self, err):
        '''
        Sends a request to the emergency_stop service to stop the robot.

        :param err: The error message to send.
        '''
        self.req.error = err
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def listener_callback(self, msg):
        '''
        Callback function for the leak/data subscription.
        If a leak is detected, this method sends a request to the emergency_stop service to stop the robot.

        :param msg: The LeakStatus message received from the leak/data topic.
        '''
        if msg.leak:
            error = "[ERROR] Leak Detected"
            self.send_request(error)


def main(args=None):
    rclpy.init(args=args)
    leak_subscriber = LeakSubscriber()
    rclpy.spin(leak_subscriber)
    leak_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()