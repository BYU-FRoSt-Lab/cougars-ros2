import launch
import launch_ros.actions
import launch_ros.descriptions

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
import datetime
import yaml

def generate_launch_description():

    config_file = "/home/frostlab/config/vehicle_config.yaml"
    with open(config_file, 'r') as f:
        vehicle_config_params = yaml.safe_load(f)

    # Get folder name from config file
    folder = vehicle_config_params['general']['bag_folder_name']
    folder = folder + "_moos_" + str(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

    # Get the directory of the launch files
    package_dir = os.path.join(
        get_package_share_directory('cougars_py'), 'launch')

    
    return launch.LaunchDescription([
        
        # Start the data recording
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', '/home/frostlab/ros2_ws/bag/' + folder, '-s', 'mcap', '-a'],
            output='screen',
        ),
        # Include additional launch files
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, 'sensors_launch.py'))
        ),
        # Start the control nodes
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='pid_control',
            parameters=[config_file],
        ),
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='moos_bridge',
            parameters=[config_file],
            output='screen',
        ),
        # Start the EmergencyStop checks
        # launch_ros.actions.Node(
        #     package='cougars_py',
        #     executable='leak_sub',
        # ),
        # launch_ros.actions.Node(
        #     package='cougars_py',
        #     executable='battery_sub',
        #     parameters=[config_file],
        # ),
    ])
