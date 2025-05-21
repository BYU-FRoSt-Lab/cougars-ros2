#!/usr/bin/env python3

# Copyright 2024 [Your Name/Organization]
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64 # For receiving depth commands

class MapPointConverterNode(Node):
    """
    A ROS 2 node that takes clicked points from Mapviz (in the 'map' frame),
    combines them with a specified depth, and republishes them as 3D
    PointStamped messages, representing relative XYZ waypoints.
    """
    def __init__(self):
        super().__init__('map_point_converter_node')

        # --- Parameters ---
        self.declare_parameter('default_depth', 0.0)  # Default depth if none is set via topic
        self.default_depth = self.get_parameter('default_depth').get_parameter_value().double_value
        self.get_logger().info(f"Default waypoint depth set to: {self.default_depth} meters")

        # --- State Variables ---
        self.current_target_depth = self.default_depth # Initialize with default
        self.last_depth_received_time = None

        # --- Quality of Service Profiles ---
        default_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS for depth setting: keep last, transient local so new subscribers get the last value
        depth_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL 
        )

        # --- Subscribers ---
        # Subscriber for points clicked in Mapviz
        self.clicked_point_subscriber = self.create_subscription(
            PointStamped,
            '/mapviz/clicked_map_point',
            self.clicked_point_callback,
            default_qos_profile)

        # Subscriber for setting the target depth for subsequent waypoints
        self.depth_subscriber = self.create_subscription(
            Float64,
            '/cougars_waypoint/set_depth', # Topic to publish desired depth to
            self.depth_callback,
            depth_qos_profile) # Use specialized QoS

        # --- Publishers ---
        # Publisher for the relative XYZ waypoints
        self.relative_goal_publisher = self.create_publisher(
            PointStamped,
            '/mapviz/relative_goal_point', 
            default_qos_profile)

        self.get_logger().info(
            f"{self.get_name()} started. "
            f"Listening for clicked points on /mapviz/clicked_map_point. "
            f"Publishing relative XYZ goals to /mapviz/relative_goal_point. "
            f"Set depth via topic /cougars_waypoint/set_depth (std_msgs/Float64)."
        )

    def depth_callback(self, msg: Float64):
        """
        Callback for receiving a new target depth.
        Depth is typically negative for underwater (e.g., -10.0 for 10 meters deep).
        """
        self.current_target_depth = msg.data
        self.last_depth_received_time = self.get_clock().now()
        self.get_logger().info(f"Target depth updated to: {self.current_target_depth:.2f} meters")

    def clicked_point_callback(self, msg: PointStamped):
        """
        Callback for receiving a PointStamped message from Mapviz.
        Combines the clicked X, Y with the current_target_depth for Z.
        """
        
        relative_goal_msg = PointStamped()
        relative_goal_msg.header.stamp = self.get_clock().now().to_msg()
        relative_goal_msg.header.frame_id = msg.header.frame_id # Should be 'map'
        relative_goal_msg.point.x = msg.point.x
        relative_goal_msg.point.y = msg.point.y
        
        # Use the latest depth received via topic, or fallback to default_depth
        # You might want to add a timeout for self.last_depth_received_time if desired
        # For underwater robots, depth is often negative (Z-axis pointing up).
        # Ensure your robot's control system interprets Z consistently.
        relative_goal_msg.point.z = self.current_target_depth 

        self.relative_goal_publisher.publish(relative_goal_msg)
        self.get_logger().info(
            f"Published relative XYZ goal: X={relative_goal_msg.point.x:.2f}, "
            f"Y={relative_goal_msg.point.y:.2f}, Z={relative_goal_msg.point.z:.2f} "
            f"in frame '{relative_goal_msg.header.frame_id}'"
        )

def main(args=None):
    rclpy.init(args=args)
    map_point_converter_node = MapPointConverterNode()
    try:
        rclpy.spin(map_point_converter_node)
    except KeyboardInterrupt:
        map_point_converter_node.get_logger().info(f"Shutting down {map_point_converter_node.get_name()}...")
    finally:
        map_point_converter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
