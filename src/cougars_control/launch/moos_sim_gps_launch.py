import launch
import launch_ros.actions
import launch_ros.descriptions
import sys

def generate_launch_description():
    '''
    :author: Braden Meyers
    :date: September 2024

    Launches the controller and any data converters needed to test nodes in simulation

    :return: The launch description.
    '''

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]

    config_file = "/home/frostlab/config/sim_params.yaml"

    return launch.LaunchDescription([
        
        # Define the namespace parameter
        launch_ros.actions.Node(
            package='cougars_control',
            executable='moos_bridge_sim',
            parameters=[config_file],
            namespace=namespace,
            output='screen',
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_controls',
            parameters=[config_file],
            namespace=namespace,
            output='screen',
        ),
    ])
