import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration

import yaml

def generate_launch_description():
    '''
    :author: Nelson Durrant
    :date: September 2024
    
    Launches the sensor nodes for the vehicle.

    :return: The launch description.
    '''

    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')

    # TODO: Figure out how to abstract this
    config_file_path = '/home/frostlab/config/vehicle_params.yaml'

    with open(config_file_path, 'r') as f:
        vehicle_config_params = yaml.safe_load(f)
    
    return launch.LaunchDescription([
        
        # Define the namespace parameter
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Unique vehicle namespace'
        ),
        # Define the config file parameter
        launch.actions.DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to the vehicle config file'
        ),
        # Launch microROS
        launch_ros.actions.Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '6000000'],
        ),
        # Set up the DVL
        launch_ros.actions.Node(
            package='dvl_a50', 
            executable='dvl_a50_sensor', 
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
        # Start the data conversion nodes
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='depth_convertor',
            parameters=[config_file],
            namespace=namespace,
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='dvl_convertor',
            parameters=[config_file],
            namespace=namespace,
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='seatrac_ahrs_convertor',
            parameters=[config_file],
            namespace=namespace,
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='gps_odom.py',
            parameters=[config_file],
            namespace=namespace,
        ),
    ])