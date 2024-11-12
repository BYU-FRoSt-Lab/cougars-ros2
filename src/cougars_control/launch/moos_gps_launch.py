import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    '''
    :author: Braden Meyers
    :date: September 2024

    Launches the controller and any data convertes needed to test nodes in simulation

    :return: The launch description.
    '''

    config_file = "/home/frostlab/config/vehicle_config.yaml"
    namespace = LaunchConfiguration('namespace')

    with open(config_file, 'r') as f:
        vehicle_config_params = yaml.safe_load(f)
    
    return launch.LaunchDescription([
        
        # Define the namespace parameter
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Unique vehicle namespace'
        ),
        # Start the control nodes
        # launch_ros.actions.Node(
        #     package='cougars_control',
        #     executable='coug_kinematics',
        #     parameters=[ParameterFile(config_file, allow_substs=True)],
        #     namespace=LaunchConfiguration('namespace'),
        # ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='moos_bridge_gps',
            parameters=[config_file],
            namespace=namespace,
            output='screen',
        ),
        # Launch microROS
        launch_ros.actions.Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '6000000'],
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='seatrac_ahrs_convertor',
            parameters=[config_file],
            namespace=namespace,
        ),
        # Setup the USBL modem
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem',
            parameters=[config_file],
            namespace=namespace,
        ),
        # Setup the GPS
        launch_ros.actions.ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container',
            name='fix_and_odometry_container',
            namespace=namespace,
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='gpsd_client',
                    plugin='gpsd_client::GPSDClientComponent',
                    name='gpsd_client',
                    namespace=namespace,
                    parameters=[vehicle_config_params[namespace]['gpsd_client']['ros__parameters']]),
                launch_ros.descriptions.ComposableNode(
                    package='gps_tools',
                    plugin='gps_tools::UtmOdometryComponent',
                    namespace=namespace,
                    name='utm_gpsfix_to_odometry_node'),
            ],
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_controls',
            parameters=[ParameterFile(config_file, allow_substs=True)],
            namespace=LaunchConfiguration('namespace'),
            output='screen',
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='gps_odom.py',
            parameters=[config_file],
            namespace=namespace,
        ),
    ])
