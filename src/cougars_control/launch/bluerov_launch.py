import sys

import launch
import launch_ros.actions
import launch_ros.descriptions

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    '''
    Launches the state estimation for the BLUEROV.

    :return: The launch description.
    '''

    sim = "false"  # Default to 'false'
    GPS = "false"  # Default to 'false'
    verbose = "false"  # Default to 'false'
    param_file = '/home/frostlab/config/vehicle_params.yaml'

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]
        if arg.startswith('param_file:='):
            param_file = arg.split(':=')[1]
        if arg.startswith("sim:="):
            sim = arg.split(":=")[1].lower()
        if arg.startswith("GPS:="):
            GPS = arg.split(":=")[1].lower()
        if arg.startswith("verbose:="):
            verbose = arg.split(":=")[1].lower()

    if verbose == "true":
        output = 'screen'
    else:
        output = 'log'

    # Get the directory of the launch files
    package_dir = os.path.join(
        get_package_share_directory('cougars_localization'), 'launch')

    
    launch_actions = []

    if sim == "false":
        sensors = launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, "sensors_launch.py"))
        )
        launch_actions.append(sensors)

    #TODO just make a parameter in yaml for moos GPS Only
    # if GPS == "true":
    #     moos = launch_ros.actions.Node(
    #         package='cougars_control',
    #         executable='moos_bridge_gps',
    #         parameters=[param_file],
    #         namespace=namespace,
    #         output=output,
    #     )
    #     launch_actions.append(moos)
    # else:
    #     moos = launch_ros.actions.Node(
    #         package='cougars_control',
    #         executable='moos_bridge',
    #         parameters=[param_file],
    #         namespace=namespace,
    #         output=output,
    #     )
    #     launch_actions.append(moos)
        


    launch_actions.extend([

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, "converters_launch.py"))
        ),   
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='factor_graph.py',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        ),
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem_pinger',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        ),
        launch_ros.actions.Node(
            package='mavlink_bridge',
            executable='mavlink_bridge',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        ),
    ])

    return launch.LaunchDescription(launch_actions)