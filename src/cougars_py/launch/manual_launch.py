import launch
import launch_ros.actions
import launch_ros.descriptions

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
        # launch.actions.ExecuteProcess(
        #     cmd=['ros2', 'topic', 'pub', 'dvl/config/command', 'dvl_msgs/msg/ConfigCommand', '{command: \'set_config\', parameter_name: \'acoustic_enabled\', parameter_value: true}', '-t', '10'],
        #     output='screen'
        # ),
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
            package='cougars_py',
            executable='manual_control',
            parameters=[config_file],
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
