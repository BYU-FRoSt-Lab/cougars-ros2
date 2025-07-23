import sys

import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    '''

    Launches the manual control and sensor nodes for the vehicle.

    :return: The launch description.
    '''

    # default parameter file paths
    param_file = '/home/frostlab/config/deploy_tmp/vehicle_params.yaml'
    fleet_param = '/home/frostlab/config/deploy_tmp/fleet_params.yaml'
    # Get the directory of the launch files
    localization_package_dir = os.path.join(
        get_package_share_directory('cougars_localization'), 'launch')
    control_package_dir = os.path.join(
        get_package_share_directory('cougars_control'), 'launch')

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coug0'
    )
    sim_launch_arg = DeclareLaunchArgument(
        'sim',
        default_value='False'
    )
    demo_launch_arg = DeclareLaunchArgument(
        'demo',
        default_value='False'
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value=param_file
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value=fleet_param
    )
    verbose_launch_arg = DeclareLaunchArgument(
        'verbose',
        default_value='False',    
    )
    if LaunchConfiguration('verbose') == 'True':
        print("Verbose mode is enabled.")
        output = 'screen'
    else:
        print("Verbose mode is disabled.")
        output = 'log'
    
    launch_actions = []

    if LaunchConfiguration('sim') == "False":
        sensors = launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization_package_dir, "sensors_launch.py"))
        )
        launch_actions.append(sensors)
    else:
        if LaunchConfiguration('demo') == "True":
            demo = launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(control_package_dir, "demo_launch.py"))
            )
            launch_actions.append(demo)

    if LaunchConfiguration('fins') == "True":
        fins_manual = launch_ros.actions.Node(
            package='cougars_control',
            executable='fins_manual.py',
            parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')], 
            namespace=LaunchConfiguration('namespace'),
            output=output,
            emulate_tty=True,
        )
        launch_actions.append(fins_manual)
    else:
        manual_mission = launch_ros.actions.Node(
            package='cougars_control',
            executable='manual_mission.py',
            parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
            namespace=LaunchConfiguration('namespace'),
            output=output,
        )
        controls = launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_controls',
            parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
            namespace=LaunchConfiguration('namespace'),
            output=output,
        )
        launch_actions.append(manual_mission)
        launch_actions.append(controls)
    
    
    launch_actions.extend([
        
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization_package_dir, "converters_launch.py"))
        ),  
        #start factor graph for logging
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='factor_graph.py',
            parameters=[LaunchConfiguration('param_file')],
            namespace=LaunchConfiguration('namespace'),
            output=output,
        ), 


        # Start the rf_bridge node
        # launch_ros.actions.Node(
        #     package='cougars_coms',
        #     executable='rf_bridge.py',
        #     parameters=[LaunchConfiguration('param_file')],
        #     namespace=LaunchConfiguration('namespace'),
        #     output=output,
        # ), 
        
    ])

    return launch.LaunchDescription(launch_actions)