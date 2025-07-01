import sys

import launch
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    '''
    :author: Nelson Durrant
    :date: September 2024
    
    Launches the sensor nodes for the vehicle.

    :return: The launch description.
    '''
    param_file = '/home/frostlab/config/vehicle_params.yaml'
    fleet_param = '/home/frostlab/config/fleet_params.yaml'
    GPS = "false"  # Default to 'false'
    namespace = ''

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]
        if arg.startswith('param_file:='):
            param_file = arg.split(':=')[1]
        if arg.startswith('fleet_param:='):
            fleet_param = arg.split(':=')[1]
        if arg.startswith("GPS:="):
            GPS = arg.split(":=")[1].lower()

    launch_actions = []

    if GPS == "false":
        depth = launch_ros.actions.Node(
            package='cougars_localization',
            executable='depth_convertor',
            parameters=[param_file, fleet_param],
            namespace=namespace,
        )
        dvl = launch_ros.actions.Node(
            package='cougars_localization',
            executable='dvl_convertor',
            parameters=[param_file, fleet_param],
            namespace=namespace,
        )
        dvl_global = launch_ros.actions.Node(
            package='cougars_localization',
            executable='dvl_global',
            parameters=[param_file, fleet_param],
            namespace=namespace,
        )
        launch_actions.extend([
            depth, 
            dvl, 
            dvl_global])



    launch_actions.extend([
        # Start the data conversion nodes
        launch_ros.actions.Node(
            package='cougars_coms',
            executable='cougars_coms',
            parameters=[param_file, fleet_param],
            namespace=namespace
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='seatrac_ahrs_convertor',
            parameters=[param_file, fleet_param],
            namespace=namespace,
        ),
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='gps_odom.py',
            parameters=[param_file, fleet_param],
            namespace=namespace,
        ),

        launch_ros.actions.Node(
            package='cougars_localization',
            executable='static_tf_publisher',
            parameters=[param_file, fleet_param],
            namespace=namespace
        )
    ])

    return launch.LaunchDescription(launch_actions)