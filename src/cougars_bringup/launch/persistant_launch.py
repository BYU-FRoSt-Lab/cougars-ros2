import sys

import launch
import launch_ros.actions
import launch_ros.descriptions

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    '''

    Launches the manual control and sensor nodes for the vehicle.

    :return: The launch description.
    '''

    # Get the directory of the launch files
    cougars_control_package_dir = os.path.join(
        get_package_share_directory('cougars_control'), 'launch')

    # TODO change this to sim_mode param
    sim = "false"  # Default to 'false'
    verbose = "false"  # Default to 'false'
    fins = "false"  # Default to 'false'
    param_file = '/home/frostlab/config/vehicle_params.yaml'
    fleet_param = '/home/frostlab/config/fleet_params.yaml'
    namespace = ''

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]
        if arg.startswith('param_file:='):
            param_file = arg.split(':=')[1]
        if arg.startswith('fleet_param:='):
            fleet_param = arg.split(':=')[1]
        if arg.startswith("sim:="):
            sim = arg.split(":=")[1].lower()
        if arg.startswith("verbose:="):
            verbose = arg.split(":=")[1].lower()
        if arg.startswith("fins:="):
            fins = arg.split(":=")[1].lower()
    
    if verbose == "true":
        output = 'screen'
    else:
        output = 'log'
    
    launch_actions = []

    manual = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cougars_control_package_dir, "manual_launch.py"))
    )
    launch_actions.append(manual)

    # TODO add more nodes here that can be activated and disactivated
    
    
    launch_actions.extend([

        launch_ros.actions.Node(
            package='cougars_bringup',
            executable='bag_recorder',
            name='bag_recorder',
            parameters=[param_file, fleet_param], #TODO 
            namespace=namespace,
            output=output,
        ),

    ])

    return launch.LaunchDescription(launch_actions)