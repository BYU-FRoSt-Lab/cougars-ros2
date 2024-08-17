"""Launch a talker and a listener in a component container."""

import os
import launch
from launch_ros.actions import ComposableNodeContainer
import launch_ros.actions
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import launch_ros
from ament_index_python.packages import get_package_share_directory
import yaml


# Path to your custom YAML configuration file
config_file = "/home/frostlab/config/vehicle_config.yaml"

# Load the parameters from the YAML file
# COULD FIX THE GPSD NODE IN THE FUTURE TO ACCEPT YAML FILE PATH
with open(config_file, 'r') as f:
    vehicle_config_params = yaml.safe_load(f)

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='fix_and_odometry_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='gpsd_client',
                    plugin='gpsd_client::GPSDClientComponent',
                    name='gpsd_client',
                    parameters=[vehicle_config_params['gpsd_client']['ros__parameters']]),
                ComposableNode(
                    package='gps_tools',
                    plugin='gps_tools::UtmOdometryComponent',
                    name='utm_gpsfix_to_odometry_node')
            ],
            output='screen',
    )

    seatrac = launch_ros.actions.Node(
            package='seatrac',
            executable='modem',
            parameters=[config_file]
    )
    converter = launch_ros.actions.Node(
            package='cougars_py',
            executable='seatrac_ahrs_converter'
    )

    transform = launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('/gps/fix', '/fix'),
                ('imu/data','/modem_imu')
            ]
           )

    rosbag = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '/fix', '/modem_imu', '/odometry/gps'
            ],
            output='screen'
    )

    return launch.LaunchDescription([container,
                                     rosbag,
                                     seatrac,
                                     transform,
                                     converter,
            # launch.actions.RegisterEventHandler(
            #     event_handler=launch.event_handlers.OnProcessExit(
            #         target_action=container,
            #         on_exit=[launch.actions.EmitEvent(
            #             event=launch.events.Shutdown())]
            #         ))
                ])

