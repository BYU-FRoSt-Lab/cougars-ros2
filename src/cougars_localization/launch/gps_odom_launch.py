import launch
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    '''
    :author: Braden Meyers
    :date: September 2024
    
    Launches the GPS to Odometry conversion node.

    :return: The launch description.
    '''

    config_file = "/home/frostlab/config/vehicle_config.yaml"
    
    return launch.LaunchDescription([
        
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='gps_odom.py',
            parameters=[config_file],
        ),
    ])