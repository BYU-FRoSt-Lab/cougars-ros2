import launch
import launch_ros.actions
import launch_ros.descriptions

import os
import yaml
from ament_index_python.packages import get_package_share_directory

gpsd_client_share_dir = get_package_share_directory('gpsd_client')
gpsd_client_params_file = os.path.join(gpsd_client_share_dir, 'config', 'gpsd_client.yaml')
with open(gpsd_client_params_file, 'r') as f:
    gpsd_client_params = yaml.safe_load(f)['gpsd_client']['ros__parameters']

def generate_launch_description():
    config_file = "/home/frostlab/config/vehicle_config.yaml"
    return launch.LaunchDescription([
        # Start recording all topics to an mcap file
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-a'],
            output='screen'
        ),
        # Set up the DVL and enable acoustics
        launch_ros.actions.Node(
            package='dvl_a50', 
            executable='dvl_a50_sensor', 
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', 'dvl/config/command', 'dvl_msgs/msg/ConfigCommand', '{command: \'set_config\', parameter_name: \'acoustic_enabled\', parameter_value: false}', '-t', '10'],
            output='screen'
        ),
        # Setup the USBL modem
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem',
            parameters=[config_file]
        ),
        # Setup the GPS
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'coug_navsat_transform.yaml')],
            remappings=[
                ('/gps/fix', '/fix'),
                ('imu/data','/modem_imu')
            ]
        ),
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
                    parameters=[gpsd_client_params]),
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
            package='cougars_cpp',
            executable='vehicle_status'
        ),
        # TODO: Add the EKF nodes
        # Start the control nodes
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='pid_control',
            parameters=[config_file]
        ),
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='moos_bridge',
            parameters=[config_file],
            output='screen'
        ),
        # Start the EmergencyStop checks
        # launch_ros.actions.Node(
        #     package='cougars_py',
        #     executable='leak_sub'
        # ),
        # launch_ros.actions.Node(
        #     package='cougars_py',
        #     executable='battery_sub'
        # ),
    ])