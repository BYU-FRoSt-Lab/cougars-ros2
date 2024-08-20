import launch
import launch_ros.actions
import launch_ros.descriptions
import os
import yaml
from ament_index_python.packages import get_package_share_directory


config_file = "/home/frostlab/config/vehicle_config.yaml"
# Load the parameters from the YAML file
# COULD FIX THE GPSD NODE IN THE FUTURE TO ACCEPT YAML FILE PATH
with open(config_file, 'r') as f:
    vehicle_config_params = yaml.safe_load(f)


def generate_launch_description():
    
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
#         launch.actions.ExecuteProcess(
#             cmd=['ros2', 'topic', 'pub', 'dvl/config/command', 'dvl_msgs/msg/ConfigCommand', '{command: \'set_config\', parameter_name: \'acoustic_enabled\', parameter_value: false}', '-t', '10'],
#             output='screen'
#         ),
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
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'coug_ekf.yaml')],
            remappings=[                        # ^^^ If we move theses parameters from coug_ekf.yaml to CougarsSetup/config/vehicle_config.yaml
                ('/gps/fix', '/fix'),           #      replace this with [config_file]
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
                    parameters=[vehicle_config_params['gpsd_client']['ros__parameters']]),
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
        # EKF nodes
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'coug_ekf.yaml')],
            remappings=[('/odometry/filtered', '/odometry/local')] 
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'coug_ekf.yaml')],
            remappings=[('/odometry/filtered', '/odometry/global')]
        ),
        # Start the control nodes
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='pid_control',
            parameters=[config_file]
        ),
#         launch_ros.actions.Node(
#             package='cougars_cpp',
#             executable='moos_bridge',
#             parameters=[config_file],
#             output='screen',
#             emulate_tty=True
#         ),
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
