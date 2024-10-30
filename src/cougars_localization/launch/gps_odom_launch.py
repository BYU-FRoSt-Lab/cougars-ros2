import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    '''
    :author: Braden Meyers
    :date: September 2024
    
    Launches the GPS to Odometry conversion node.

    :return: The launch description.
    '''

    config_file = "/home/frostlab/config/vehicle_config.yaml"
    
    return launch.LaunchDescription([
        
        # Define the namespace parameter
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Unique vehicle namespace'
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='gps_odom.py',
            parameters=[config_file],
            namespace=LaunchConfiguration('namespace')
        ),
    ])