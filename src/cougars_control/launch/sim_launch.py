import launch
import launch_ros.actions
import launch_ros.descriptions

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    '''
    :author: Braden Meyers
    :date: September 2024

    Launches the controller and any data convertes needed to test nodes in simulation

    :return: The launch description.
    '''

    config_file = "~/config/sim_config.yaml"

    # Get the directory of the launch files
    package_dir = os.path.join(
        get_package_share_directory('cougars_localization'), 'launch')


    return launch.LaunchDescription([
        
        # Start the control nodes
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_kinematics',
            parameters=[config_file],
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_controls',
            parameters=[config_file],
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='manual_mission.py',
            parameters=[config_file],
        ),
    ])
