import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory
import os
import datetime
import sys

for arg in sys.argv:
    if arg.startswith("folder:="):
        folder = arg.split(":=")[1]

def generate_launch_description():

    folder_exists = True
    while folder_exists:
        folder = folder + "_" + str(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
        if not os.path.exists("/home/frostlab/ros2_ws/bag/" + folder):
            folder_exists = False

    # Get the directory of the launch files
    package_dir = os.path.join(
        get_package_share_directory('cougars_py'), 'launch')

    config_file = "/home/frostlab/config/vehicle_config.yaml"

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
            package='cougars_py',
            executable='manual_control',
            parameters=[config_file],
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
