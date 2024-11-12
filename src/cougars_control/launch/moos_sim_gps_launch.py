import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile

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

    config_file = "/home/frostlab/config/sim_config.yaml"

    return launch.LaunchDescription([
        
        # Define the namespace parameter
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Unique vehicle namespace'
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='moos_bridge_sim',
            parameters=[config_file],
            namespace=LaunchConfiguration('namespace'),
            output='screen',
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_controls',
            parameters=[ParameterFile(config_file, allow_substs=True)],
            namespace=LaunchConfiguration('namespace'),
            output='screen',
        ),
    ])
