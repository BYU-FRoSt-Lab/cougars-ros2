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

    param_file = "/home/frostlab/config/sim_params.yaml"

    return launch.LaunchDescription([
        # Start the control nodes
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_kinematics',
            parameters=[param_file],
            namespace=namespace,
            output='screen',
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_controls',
            parameters=[param_file],
            namespace=namespace,
            output='log',
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='dvl_global',
            parameters=[param_file],
            namespace=namespace,
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='factor_graph.py',
            parameters=[param_file],
            namespace=namespace,
            output='log',
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='manual_mission.py',
            parameters=[param_file],
            namespace=namespace,
            output='screen',
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='depth_convertor',
            parameters=[param_file],
            namespace=namespace,
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='gps_odom.py',
            parameters=[param_file],
            namespace=namespace,
        ),
    ])
