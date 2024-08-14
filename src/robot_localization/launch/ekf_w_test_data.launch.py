

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_dir = get_package_share_directory("robot_localization")
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'params', 'coug_ekf.yaml')],
            remappings=[
                ("gps_filtered", "/holoocean/GPSSensor"),
                ("modem_orientation", "/holoocean/DynamicsSensorOdom"),
                ("dvl_dead_reckoning", "/holoocean/dead_reckon")
            ]
        ),
        # launch_ros.actions.Node(
        #     package='cougars_py',
        #     executable='ekf_plotter',
        #     name='ekf_plotter',
        #     output='screen'
        # ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/claytonsmith/Documents/CougarsRPi/src/robot_localization/rosbag2_2024_08_14-10_58_26'],
            output='screen'
        ),
        # launch.actions.ExecuteProcess(
        #     cmd=['ros2', 'topic', 'echo', '/odometry/filtered'],
        #     output='screen'
        # )
    ])
