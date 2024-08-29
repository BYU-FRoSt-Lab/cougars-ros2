import launch
from launch.actions import ExecuteProcess
import launch_ros.actions
import launch_ros.descriptions

import os
import yaml
import datetime


def generate_launch_description():

    config_file = "/home/frostlab/config/vehicle_config.yaml"
    with open(config_file, 'r') as f:
        vehicle_config_params = yaml.safe_load(f)
    
    return launch.LaunchDescription([

        folder_exists = True
        while folder_exists:
            folder = input("Enter a new descriptive folder name: ")
            folder = folder + "_" + str(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
            if not os.path.exists("/home/frostlab/ros2_ws/bag/" + folder):
                folder_exists = False
        
        # Start the data recording
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', '/home/frostlab/ros2_ws/bag/' + folder, '-s', 'mcap', '-a'],
            output='screen',
        ),
        # Set up the DVL
        launch_ros.actions.Node(
            package='dvl_a50', 
            executable='dvl_a50_sensor', 
        ),
        # Setup the USBL modem
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem',
            parameters=[config_file]
        ),
        # Setup the GPS
        launch_ros.actions.ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container',
            name='fix_and_odometry_container',
            namespace='',
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='gpsd_client',
                    plugin='gpsd_client::GPSDClientComponent',
                    name='gpsd_client',
                    parameters=[vehicle_config_params['/gpsd_client']['ros__parameters']]),
                launch_ros.descriptions.ComposableNode(
                    package='gps_tools',
                    plugin='gps_tools::UtmOdometryComponent',
                    name='utm_gpsfix_to_odometry_node')
            ],
        ),
        # Start the data conversion nodes
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='depth_convertor'
        ),
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='dvl_convertor'
        ),
        launch_ros.actions.Node(
            package='cougars_py',
            executable='seatrac_ahrs_converter'
        ),
        launch_ros.actions.Node(
            package='cougars_py',
            executable='gps_odom',
            name='gps_odom',
            parameters=[config_file],
        ),
        # launch_ros.actions.Node(
        #     package='cougars_cpp',
        #     executable='vehicle_status'
        # ),
    ])