#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from frost_interfaces.srv import EmergencyStop
from frost_interfaces.msg import LeakStatus
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType




class LeakSubscriber(Node):
    '''
    :author: Nelson Durrant
    :date: Revised Jan 2025
    
    Revised to just change the leak_detection parameter (Jan 2025)
    
    Subscribes:
        - leak/data (frost_interfaces/msg/LeakStatus)
 
    '''
    def __init__(self):

        '''
        Creates a new LeakSubscriber node.
        '''
        super().__init__("leak_subscriber")

        '''
        Subscription to the "leak/data" topic with the message type LeakStatus.
        '''

        self.subscription = self.create_subscription(
            LeakStatus, "leak/data", self.listener_callback, qos_profile_sensor_data
        )
        
        self.subscription  # prevent unused variable warning
        
        self.cli = self.create_client(SetParameters, 'emergency_protocols/set_parameters')  

        while not self.cli.wait_for_service(timeout_sec=1):
            self.get_logger().warn("Setting leak parameter service not available, waiting...")
        self.req = SetParameters.Request()

        

    def listener_callback(self, msg):
        '''
        sets the leak_detected param
        '''

        if msg.leak:
            leak_param = Parameter()
            leak_param.name = 'leak_detected'
            leak_param.value.type = ParameterType.PARAMETER_BOOL
            leak_param.value.bool_value = True
            self.req.parameters.append(leak_param)
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)

            self.get_logger().info("%s" % (self.future.result()))
            if self.future.result() == False:
                self.get_logger().error("Failed to notify of leak, emergency leak_param not changed")

            

            




def main(args=None):
    rclpy.init(args=args)
    leak_subscriber = LeakSubscriber()
    rclpy.spin(leak_subscriber)
    leak_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()