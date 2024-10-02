import launch
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():

    config_file = "/home/frostlab/config/vehicle_config.yaml"
    
    return launch.LaunchDescription([
        
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