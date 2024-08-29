import launch
import launch_ros.actions
import launch_ros.descriptions

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_file = "/home/frostlab/config/vehicle_config.yaml"

    # COULD FIX THE GPSD NODE IN THE FUTURE TO ACCEPT YAML FILE PATH
    with open(config_file, 'r') as f:
        vehicle_config_params = yaml.safe_load(f)
    
    return launch.LaunchDescription([
        # Start recording all topics to an mcap file
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-a'],
            output='screen'
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
            executable='depth_convertor',
        ),
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='dvl_convertor',
        ),
        launch_ros.actions.Node(
            package='cougars_py',
            executable='seatrac_ahrs_converter',
        ),
        # launch_ros.actions.Node(
        #     package='cougars_cpp',
        #     executable='vehicle_status',
        # ),
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
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='dvl_config',
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
