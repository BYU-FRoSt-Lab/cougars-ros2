import launch
import launch_ros.actions
import launch_ros.descriptions
import yaml

config_file = "/home/frostlab/config/vehicle_config.yaml"
# Load the parameters from the YAML file
# COULD FIX THE GPSD NODE IN THE FUTURE TO ACCEPT YAML FILE PATH
with open(config_file, 'r') as f:
    vehicle_config_params = yaml.safe_load(f)

def generate_launch_description():
    return launch.LaunchDescription([
        # Start recording all topics to an mcap file
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', '/home/frostlab/ros2_ws/bag', '-s', 'mcap', '-a'],
            output='screen',

        ),
        # Set up the DVL and enable acoustics
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
        # Start the data conversion nodes
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='depth_convertor'
        ),
        launch_ros.actions.Node(
            package='cougars_py',
            executable='gps_odom',
            name='gps_odom',
            parameters=[config_file],
        ),
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='dvl_convertor'
        ),
        launch_ros.actions.Node(
            package='cougars_py',
            executable='seatrac_ahrs_converter'
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
        # launch_ros.actions.Node(
        #     package='cougars_cpp',
        #     executable='vehicle_status'
        # ),
        # TODO: Add the EKF nodes
    ])