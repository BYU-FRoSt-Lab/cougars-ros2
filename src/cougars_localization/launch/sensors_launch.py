import sys

import launch
import launch_ros.actions
import launch_ros.descriptions

import yaml

def generate_launch_description():
    '''
    :author: Nelson Durrant
    :date: September 2024
    
    Launches the sensor nodes for the vehicle.

    :return: The launch description.
    '''

    param_file = '/home/frostlab/config/vehicle_params.yaml'
    GPS = "false"  # Default to 'false'
    verbose = "false"

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]
        if arg.startswith('param_file:='):
            param_file = arg.split(':=')[1]
        if arg.startswith("verbose:="):
            verbose = arg.split(":=")[1].lower()
        if arg.startswith("GPS:="):
            GPS = arg.split(":=")[1].lower()
    
    if verbose == "true":
        output = 'screen'
    else:
        output = 'log'

    launch_actions = []

    if GPS == "false":
        dvl = launch_ros.actions.Node(
            package='dvl_a50', 
            executable='dvl_a50_sensor', 
            namespace=namespace,
        )
        launch_actions.append(dvl)


    with open(param_file, 'r') as f:
        vehicle_params = yaml.safe_load(f)
    
    launch_actions.extend([
        
        # # Launch microROS
        # launch_ros.actions.Node(
        #     package='micro_ros_agent',
        #     executable='micro_ros_agent',
        #     arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '6000000'],
        # ),

        # Serial Teensy connection
        launch_ros.actions.Node(
            package='fin_sub_cpp', 
            executable='control_node', 
            namespace=namespace,
            output='log',
        ),
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_kinematics',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        ),
        # Setup the USBL modem
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        ),
        # Setup the GPS
        launch_ros.actions.ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container',
            name='fix_and_odometry_container',
            namespace=namespace,
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='gpsd_client',
                    plugin='gpsd_client::GPSDClientComponent',
                    name='gpsd_client',
                    namespace=namespace,
                    parameters=[
                        vehicle_params[namespace]['gpsd_client']['ros__parameters'],
                        {'log_level': 'warn'}  # Add log level here
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                launch_ros.descriptions.ComposableNode(
                    package='gps_tools',
                    plugin='gps_tools::UtmOdometryComponent',
                    namespace=namespace,
                    name='utm_gpsfix_to_odometry_node',
                    parameters=[
                        {'log_level': 'warn'}  # Add log level here
                    ],
                ),
            ],
            output=output,
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
    ])

    return launch.LaunchDescription(launch_actions)