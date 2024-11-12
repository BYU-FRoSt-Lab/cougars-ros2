import sys

import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    '''
    :author: Nelson Durrant
    :date: September 2024

    Launches the manual control and sensor nodes for the vehicle.

    :return: The launch description.
    '''

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]
        if arg.startswith('param_file:='):
            param_file = arg.split(':=')[1]

    # Get the directory of the launch files
    package_dir = os.path.join(
        get_package_share_directory('cougars_localization'), 'launch')

    return launch.LaunchDescription([
        
        # Include additional launch files
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, 'sensors_launch.py'))
        ),
        # Start the control nodes
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_kinematics',
            parameters=[param_file],
            namespace=namespace,
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_controls',
            parameters=[param_file],
            namespace=namespace,
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='manual_mission.py',
            parameters=[param_file],
            namespace=namespace,
        ),
        # Start the EmergencyStop checks
        launch_ros.actions.Node(
            package='cougars_control',
            executable='leak_sub.py',
            parameters=[param_file],
            namespace=namespace,
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='battery_sub.py',
            parameters=[param_file],
            namespace=namespace,
        ),
    ])
