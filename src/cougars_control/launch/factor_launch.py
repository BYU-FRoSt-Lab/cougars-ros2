import sys
import rclpy
import launch
import launch_ros.actions
import launch_ros.descriptions

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
BAGNAME = '5.0_SO-2025-05-01-19-38-59/'
BAGDIR="/home/frostlab/bag/"
NS='/coug1'


def generate_launch_description():
    '''

    Launches the manual control and sensor nodes for the vehicle.

    :return: The launch description.
    '''

    verbose = "true"  # Default to 'false'

    namespace = NS

    if verbose == "true":
        output = 'screen'
    else:
        output = 'log'
    
    param_file = '/home/frostlab/config/vehicle_params.yaml'

    launch_actions = []

    launch_actions.extend([
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='factor_graph.py',
            namespace=namespace,
            output=output,
        ),
        launch.actions.ExecuteProcess(
            cmd=["ros2", "bag", "play", BAGDIR+BAGNAME, "--clock", "-r 5"],
            output=output
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='dummy_factor_starter.py',
            namespace=namespace,
            output=output,
        ),
        launch.actions.TimerAction(
            period=2.0, #record after 2 seconds
            actions= [launch.actions.ExecuteProcess(cmd=["ros2", "bag", "record", NS+"/smoothed_output"],output=output)]
        )
    ])
    return launch.LaunchDescription(launch_actions)