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
    package_dir = os.path.join(
        get_package_share_directory('cougars_localization'), 'launch')

    sim = "false"  # Default to 'false'
    verbose = "false"  # Default to 'false'
    fins = "false"  # Default to 'false'
    param_file = '/home/frostlab/config/vehicle_params.yaml'
    namespace = ''

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]
        if arg.startswith('param_file:='):
            param_file = arg.split(':=')[1]
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

    if sim == "false":
        sensors = launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, "sensors_launch.py"))
        )
        launch_actions.append(sensors)

    if fins == "true":
        fins_manual = launch_ros.actions.Node(
            package='cougars_control',
            executable='fins_manual.py',
            parameters=[param_file],
            namespace=namespace,
            output=output,
            emulate_tty=True,
        )
        launch_actions.append(fins_manual)
    else:
        manual_mission = launch_ros.actions.Node(
            package='cougars_control',
            executable='manual_mission.py',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        )
        controls = launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_controls',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        )
        launch_actions.append(manual_mission)
        launch_actions.append(controls)
    
    
    launch_actions.extend([
        
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, "converters_launch.py"))
        ),   
        # Start the EmergencyStop checks
        # launch_ros.actions.Node(
        #     package='cougars_control',
        #     executable='leak_sub.py',
        #     parameters=[param_file],
        #     namespace=namespace,
        # ),
        # launch_ros.actions.Node(
        #     package='cougars_control',
        #     executable='battery_sub.py',
        #     parameters=[param_file],
        #     namespace=namespace,
        # ),


        launch_ros.actions.Node(
            package='cougars_control',
            executable='emergency_protocols',
            parameters=[param_file],
            namespace=namespace,
        ),

        


    ])

    return launch.LaunchDescription(launch_actions)