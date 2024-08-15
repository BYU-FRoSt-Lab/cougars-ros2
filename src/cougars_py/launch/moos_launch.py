import launch
import launch_ros.actions

def generate_launch_description():
    config_file = "/home/frostlab/config/vehicle_config.yaml"
    return launch.LaunchDescription([
        # Start recording all topics to an mcap file
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-a'],
            output='screen'
        ),
        # Set up the DVL and enable acoustics
        launch.actions.DeclareLaunchArgument('ip_address', default_value='192.168.194.95'),
        launch_ros.actions.Node(
            package='dvl_a50', 
            executable='dvl_a50_sensor', 
            parameters=[{'dvl_ip_address': launch.substitutions.LaunchConfiguration('ip_address')}],
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-1', 'dvl/config/command', 'dvl_msgs/msg/ConfigCommand', '{command: \'set_config\', parameter_name: \'acoustic_enabled\', parameter_value: true\'}', '--once'],
            output='screen'
        ),
        # TODO: Add the modem and GPS nodes
        # Start the data conversion nodes
        # TODO: Add more of these
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='depth_convertor'
        ),
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='dvl_convertor'
        ),
        # Start the control nodes
        launch_ros.actions.Node(
            package='cougars_py',
            executable='leak_sub'
        ),
        launch_ros.actions.Node(
            package='cougars_py',
            executable='battery_sub'
        ),
        launch_ros.actions.Node(
            package='cougars_cpp',
            executable='pid_control',
            parameters=[config_file]
        ),
        launch_ros.actions.Node(
            package='cougars_py',
            executable='moos_bridge',
            parameters=[config_file]
        ),
    ])