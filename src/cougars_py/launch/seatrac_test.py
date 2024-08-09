import launch
import launch_ros.actions

def generate_launch_description():
    config_file = "~/config/vehicle_config.yaml"
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cougars_py',
            executable='modem_pinger',
            parameters=[config_file]
        ),
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem',
            parameters=[config_file]
        ),

    ])
