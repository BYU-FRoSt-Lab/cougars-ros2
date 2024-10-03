import rclpy
from rclpy.node import Node
from frost_interfaces.msg import DesiredDepth, DesiredHeading, DesiredSpeed
from frost_interfaces.srv import EmergencyStop
from std_msgs.msg import Empty
from rclpy.qos import qos_profile_system_default


# TODO: Add to cougars_control package

class ManualControl(Node):
    '''
    :author: Nelson Durrant
    :date: September 2024

    A simple ROS2 node that publishes desired depth, heading, and speed values to control the vehicle.
    The desired values are set based on a simple state machine that transitions between three states.

    Subscribes:
        - init (std_msgs/msg/Empty)
    Publishes:
        - desired_depth (frost_interfaces/msg/DesiredDepth)
        - desired_heading (frost_interfaces/msg/DesiredHeading)
        - desired_speed (frost_interfaces/msg/DesiredSpeed)
    '''
    def __init__(self):
        '''
        Creates a new ManualControl node.
        '''
        super().__init__("manual_control")

        # Declare parameters
        self.declare_parameter('vehicle_id', 0)
        '''
        :param vehicle_id: The ID of the vehicle. The default value is 0.
        '''

        self.declare_parameter('command_timer_period', 0.5) # in seconds
        '''
        :param command_timer_period: The period at which the state machine updates the desired values. The default value is 0.5 seconds.
        '''

        self.declare_parameter('state_1_count', 0) # in intervals based on command_timer_period
        '''
        :param state_1_count: The number of intervals for state 1, based on the command_timer_period. The default value is 0.
        '''

        self.declare_parameter('state_1_depth', 0.0)
        '''
        :param state_1_depth: The desired depth value for state 1. The default value is 0.0.
        '''

        self.declare_parameter('state_1_heading', 0.0)
        '''
        :param state_1_heading: The desired heading value for state 1. The default value is 0.0.
        '''

        self.declare_parameter('state_1_speed', 0.0)
        '''
        :param state_1_speed: The desired speed value for state 1. The default value is 0.0.
        '''

        self.declare_parameter('state_2_count', 0) # in intervals based on command_timer_period
        '''
        :param state_2_count: The number of intervals for state 2, based on the command_timer_period. The default value is 0.
        '''

        self.declare_parameter('state_2_depth', 0.0)
        '''
        :param state_2_depth: The desired depth value for state 2. The default value is 0.0.
        '''

        self.declare_parameter('state_2_heading', 0.0)
        '''
        :param state_2_heading: The desired heading value for state 2. The default value is 0.0.
        '''

        self.declare_parameter('state_2_speed', 0.0)
        '''
        :param state_2_speed: The desired speed value for state 2. The default value is 0.0.
        '''

        self.declare_parameter('state_3_count', 0) # in intervals based on command_timer_period
        '''
        :param state_3_count: The number of intervals for state 3, based on the command_timer_period. The default value is 0.
        '''

        self.declare_parameter('state_3_depth', 0.0)
        '''
        :param state_3_depth: The desired depth value for state 3. The default value is 0.0.
        '''

        self.declare_parameter('state_3_heading', 0.0)
        '''
        :param state_3_heading: The desired heading value for state 3. The default value is 0.0.
        '''

        self.declare_parameter('state_3_speed', 0.0)
        '''
        :param state_3_speed: The desired speed value for state 3. The default value is 0.0.
        '''

        # Create the publishers
        self.depth_publisher = self.create_publisher(
            DesiredDepth,
            "desired_depth",
            qos_profile_system_default
        )
        '''
        Create a publisher for the "desired_depth" topic with the message type DesiredDepth.
        '''

        self.heading_publisher = self.create_publisher(
            DesiredHeading,
            "desired_heading",
            qos_profile_system_default
        )
        '''
        Create a publisher for the "desired_heading" topic with the message type DesiredHeading.
        '''

        self.speed_publisher = self.create_publisher(
            DesiredSpeed,
            "desired_speed",
            qos_profile_system_default
        )
        '''
        Create a publisher for the "desired_speed" topic with the message type DesiredSpeed.
        '''

        # Create the subscriptions
        self.subscription = self.create_subscription(
            Empty, 
            "/init", 
            self.listener_callback, 
            qos_profile_system_default
        )
        self.subscription  # prevent unused variable warning
        '''
        Create a subscription to the "init" topic with the message type Empty.
        '''

        # Create the timers
        self.timer = self.create_timer(
            self.get_parameter("command_timer_period").get_parameter_value().double_value,
            self.timer_callback
        )
        '''
        Create a timer that calls the timer_callback method at the specified period.
        '''

        # Create the services
        self.srv = self.create_service(
            EmergencyStop,
            "emergency_stop",
            self.emergency_stop_callback
        )
        '''
        Create a service for the "emergency_stop" service with the service type EmergencyStop.
        '''

        self.counter = 0
        self.stopped = True

    def listener_callback(self, msg):
        '''
        Callback function for the init subscription.
        Sets the stopped flag to False when the init message is received.

        :param msg: The Empty message received from the init topic.
        '''
        self.get_logger().info("[INFO] Init message received")
        self.stopped = False


    def timer_callback(self):
        '''
        Callback function for the timer.
        Runs the state machine and high-level controller, and publishes the desired depth, heading, and speed values.
        For a faster update time, adjust the PUB_TIMER_PERIOD.
        '''

        depth_msg = DesiredDepth()
        heading_msg = DesiredHeading()
        speed_msg = DesiredSpeed()

        # TODO: Adjust this simple state machine
        if not self.stopped and self.counter < self.get_parameter("state_1_count").get_parameter_value().integer_value:
            depth_msg.desired_depth = self.get_parameter("state_1_depth").get_parameter_value().double_value
            heading_msg.desired_heading = self.get_parameter("state_1_heading").get_parameter_value().double_value
            speed_msg.desired_speed = self.get_parameter("state_1_speed").get_parameter_value().double_value

            self.counter += 1 # DO NOT DELETE THIS OR BAD THINGS WILL HAPPEN - NELSON

        elif not self.stopped and self.counter < self.get_parameter("state_1_count").get_parameter_value().integer_value + self.get_parameter("state_2_count").get_parameter_value().integer_value:
            depth_msg.desired_depth = self.get_parameter("state_2_depth").get_parameter_value().double_value
            heading_msg.desired_heading = self.get_parameter("state_2_heading").get_parameter_value().double_value
            speed_msg.desired_speed = self.get_parameter("state_2_speed").get_parameter_value().double_value

            self.counter += 1 # DO NOT DELETE THIS OR BAD THINGS WILL HAPPEN - NELSON

        elif not self.stopped and self.counter < self.get_parameter("state_1_count").get_parameter_value().integer_value + self.get_parameter("state_2_count").get_parameter_value().integer_value + self.get_parameter("state_3_count").get_parameter_value().integer_value:
            depth_msg.desired_depth = self.get_parameter("state_3_depth").get_parameter_value().double_value
            heading_msg.desired_heading = self.get_parameter("state_3_heading").get_parameter_value().double_value
            speed_msg.desired_speed = self.get_parameter("state_3_speed").get_parameter_value().double_value

            self.counter += 1 # DO NOT DELETE THIS OR BAD THINGS WILL HAPPEN - NELSON

        else:
            depth_msg.desired_depth = 0.0
            heading_msg.desired_heading = 0.0
            speed_msg.desired_speed = 0.0
        
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        heading_msg.header.stamp = self.get_clock().now().to_msg()
        speed_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the messages
        self.depth_publisher.publish(depth_msg)
        self.heading_publisher.publish(heading_msg)
        self.speed_publisher.publish(speed_msg)

        self.get_logger().info("[INFO] Depth: %f, Heading: %f, Speed: %f" % (
            depth_msg.desired_depth,
            heading_msg.desired_heading,
            speed_msg.desired_speed,
        ))

    # Logs when EmergencyStop is requested
    def emergency_stop_callback(self, request, response):
        '''
        Callback function for the emergency_stop service.
        Logs the error message and sets the stopped flag to True.
        
        :param request: The EmergencyStop request message from the client.
        :param response: The EmergencyStop response message.
        '''

        self.get_logger().info("[ERROR] EMERGENCY STOP EXECUTED")
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
