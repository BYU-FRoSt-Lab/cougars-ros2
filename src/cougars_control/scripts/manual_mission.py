#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from frost_interfaces.msg import DesiredDepth, DesiredHeading, DesiredSpeed
from std_srvs.srv import SetBool
from rclpy.qos import qos_profile_system_default


class ManualMission(Node):
    '''
    :author: Nelson Durrant
    :date: September 2024

    A simple ROS2 node that publishes desired depth, heading, and speed values to control the vehicle.
    The desired values are set based on a simple state machine that transitions between three states.

    Publishes:
        - desired_depth (frost_interfaces/msg/DesiredDepth)
        - desired_heading (frost_interfaces/msg/DesiredHeading)
        - desired_speed (frost_interfaces/msg/DesiredSpeed)
        
    Services:
        - init (std_srvs/srv/Empty) 
    '''
    def __init__(self):
        '''
        Creates a new ManualMission node.
        '''
        super().__init__("manual_mission")

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
        Publisher for the "desired_depth" topic with the message type DesiredDepth.
        '''

        self.heading_publisher = self.create_publisher(
            DesiredHeading,
            "desired_heading",
            qos_profile_system_default
        )
        '''
        Publisher for the "desired_heading" topic with the message type DesiredHeading.
        '''

        self.speed_publisher = self.create_publisher(
            DesiredSpeed,
            "desired_speed",
            qos_profile_system_default
        )
        '''
        Publisher for the "desired_speed" topic with the message type DesiredSpeed.
        '''

        # Create the timers
        self.timer = self.create_timer(
            self.get_parameter("command_timer_period").get_parameter_value().double_value,
            self.timer_callback
        )
        '''
        Timer that calls the timer_callback method at the specified period.
        '''

        # Create the services
        self.srv = self.create_service(
            SetBool,
            "init_manual",
            self.listener_callback
        )
        '''
        Service for the "init_manual" topic with the service type SetBool.
        '''

        self.counter = 0
        self.started = False

        self.last_depth = -1.0
        self.last_heading = -1.0
        self.last_speed = -1.0

        self.get_parameters()

        

    def get_parameters(self):
        self.state_1_count = self.get_parameter("state_1_count").get_parameter_value().integer_value
        self.state_1_depth = self.get_parameter("state_1_depth").get_parameter_value().double_value
        self.state_1_heading = self.get_parameter("state_1_heading").get_parameter_value().double_value
        self.state_1_speed = self.get_parameter("state_1_speed").get_parameter_value().double_value

        self.state_2_count = self.get_parameter("state_2_count").get_parameter_value().integer_value + self.state_1_count
        self.state_2_depth = self.get_parameter("state_2_depth").get_parameter_value().double_value
        self.state_2_heading = self.get_parameter("state_2_heading").get_parameter_value().double_value
        self.state_2_speed = self.get_parameter("state_2_speed").get_parameter_value().double_value

        self.state_3_count = self.get_parameter("state_3_count").get_parameter_value().integer_value + self.state_2_count
        self.state_3_depth = self.get_parameter("state_3_depth").get_parameter_value().double_value
        self.state_3_heading = self.get_parameter("state_3_heading").get_parameter_value().double_value
        self.state_3_speed = self.get_parameter("state_3_speed").get_parameter_value().double_value

        self.destroy_timer(self.timer)

        # Create a new timer with the updated period
        self.timer = self.create_timer(self.get_parameter("command_timer_period").get_parameter_value().double_value, self.timer_callback)

        self.get_logger().info("Manual Mission Parameters Updated!")

    
    def listener_callback(self, request, response):
        '''
        Callback function for the init service.
        Sets the started flag to False when the init request is true.

        Sets the started flag to True and resets the counter when request is false

        '''
        init_bool = request.data
        if init_bool:
            if self.started:
                response.success = False
                response.message = 'Manual Mission has already been started. Needs to be reset before initialization'
            else:
                self.get_parameters()
                self.started = request.data
                response.success = True
                response.message = 'Manual Mission Started'
                self.counter = 0
        else:
            self.started = False
            self.counter = 0
            response.success = True
            response.message = 'Manual Mission Restarted'

        return response


    def timer_callback(self):
        '''
        Callback function for the timer.
        Runs the state machine and high-level controller, and publishes the desired depth, heading, and speed values.
        For a faster update time, adjust the command_timer_period parameter.
        '''

        depth_msg = DesiredDepth()
        heading_msg = DesiredHeading()
        speed_msg = DesiredSpeed()

        # TODO: Adjust this simple state machine
        if self.started and self.counter < self.state_1_count:
            depth_msg.desired_depth = self.state_1_depth
            heading_msg.desired_heading = self.state_1_heading
            speed_msg.desired_speed = self.state_1_speed

            self.counter += 1 # DO NOT DELETE THIS OR BAD THINGS WILL HAPPEN - NELSON

        elif self.started and self.counter < self.state_2_count:
            depth_msg.desired_depth = self.state_2_depth
            heading_msg.desired_heading = self.state_2_heading
            speed_msg.desired_speed = self.state_2_speed

            self.counter += 1 # DO NOT DELETE THIS OR BAD THINGS WILL HAPPEN - NELSON

        elif self.started and self.counter < self.state_3_count:
            depth_msg.desired_depth = self.state_3_depth
            heading_msg.desired_heading = self.state_3_heading
            speed_msg.desired_speed = self.state_3_speed

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

        # Publish the values if they change
        if depth_msg.desired_depth != self.last_depth or heading_msg.desired_heading != self.last_heading or speed_msg.desired_speed != self.last_speed:
            self.get_logger().info("Depth: %f, Heading: %f, Speed: %f" % (
                depth_msg.desired_depth,
                heading_msg.desired_heading,
                speed_msg.desired_speed,
            )
        )
            
        # Save last values
        self.last_depth = depth_msg.desired_depth
        self.last_heading = heading_msg.desired_heading
        self.last_speed = speed_msg.desired_speed


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    manual_mission = ManualMission()
    executor.add_node(manual_mission)
    executor.spin()
    executor.shutdown()
    manual_mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
