import sys

import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    '''
    Launches the state estimation for the BLUEROV.

    :return: The launch description.
    '''

    sim = "false"  # Default to 'false'
    GPS = "false"  # Default to 'false'
    verbose = "false"  # Default to 'false'
    param_file = '/home/frostlab/config/vehicle_params.yaml'
    namespace = ''
    with open(param_file, 'r') as f:
        vehicle_params = yaml.safe_load(f)

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]
        if arg.startswith('param_file:='):
            param_file = arg.split(':=')[1]
        if arg.startswith("sim:="):
            sim = arg.split(":=")[1].lower()
        if arg.startswith("GPS:="):
            GPS = arg.split(":=")[1].lower()
        if arg.startswith("verbose:="):
            verbose = arg.split(":=")[1].lower()

    if verbose == "true":
        output = 'screen'
    else:
        output = 'log'

    # Get the directory of the launch files
    package_dir = os.path.join(
        get_package_share_directory('cougars_localization'), 'launch')
    imu_package_dir = os.path.join(get_package_share_directory('microstrain_inertial_driver'), 'launch')
    dvl_package_dir = os.path.join(get_package_share_directory('dvl_a50'), 'launch')

    
    launch_actions = []

    # if sim == "false":
    #     sensors = launch.actions.IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(package_dir, "sensors_launch.py"))
    #     )
    #     launch_actions.append(sensors)

    #TODO just make a parameter in yaml for moos GPS Only
    # if GPS == "true":
    #     moos = launch_ros.actions.Node(
    #         package='cougars_control',
    #         executable='moos_bridge_gps',
    #         parameters=[param_file],
    #         namespace=namespace,
    #         output=output,
    #     )
    #     launch_actions.append(moos)
    # else:
    #     moos = launch_ros.actions.Node(
    #         package='cougars_control',
    #         executable='moos_bridge',
    #         parameters=[param_file],
    #         namespace=namespace,
    #         output=output,
    #     )
    #     launch_actions.append(moos)
        


    launch_actions.extend([
        DeclareLaunchArgument(
            'param_file',
            default_value='/home/frostlab/config/microstrain_params.yaml',
            description='Path to the microstrain parameter file'
        ),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(imu_package_dir, "microstrain_launch.py")
            ),
            launch_arguments={
                'param_file': LaunchConfiguration('param_file'),
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_dir, "converters_launch.py"))
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(dvl_package_dir, "dvl_a50.launch.py"))
        ),  
        # launch_ros.actions.Node(
        #     package='dvl_a50', 
        #     executable='dvl_a50_sensor', 
        #     parameters=[param_file],
        #     namespace=namespace,
        # ),
        # launch_ros.actions.Node(
        #     package='cougars_localization',
        #     executable='factor_graph.py',
        #     parameters=[param_file],
        #     namespace=namespace,
        #     output=output,
        # ),
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem_pinger',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        ),
        launch_ros.actions.Node(
            package='mavlink_bridge',
            executable='mavlink_bridge',
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
        # launch_ros.actions.ComposableNodeContainer(
        #     package='rclcpp_components',
        #     executable='component_container',
        #     name='fix_and_odometry_container',
        #     namespace=namespace,
        #     composable_node_descriptions=[
        #         launch_ros.descriptions.ComposableNode(
        #             package='gpsd_client',
        #             plugin='gpsd_client::GPSDClientComponent',
        #             name='gpsd_client',
        #             namespace=namespace,
        #             parameters=[
        #                 vehicle_params[namespace]['gpsd_client']['ros__parameters'],
        #                 {'log_level': 'warn'}  # Add log level here
        #             ],
        #             extra_arguments=[{'use_intra_process_comms': True}]
        #         ),
        #         launch_ros.descriptions.ComposableNode(
        #             package='gps_tools',
        #             plugin='gps_tools::UtmOdometryComponent',
        #             namespace=namespace,
        #             name='utm_gpsfix_to_odometry_node',
        #             parameters=[
        #                 {'log_level': 'warn'}  # Add log level here
        #             ],
        #         ),
        #     ],
        #     output=output,
        #     arguments=['--ros-args', '--log-level', 'WARN'],
        # ),
        
        launch_ros.actions.Node(
            package='cougars_localization',
            executable='nmea_constructor.py',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        ),
        launch_ros.actions.Node(
            package='pressure_sensor',
            executable='pressure_pub',
            parameters=[param_file],
            namespace=namespace,
            output=output,
        ),

    ])

    return launch.LaunchDescription(launch_actions)