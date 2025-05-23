import sys
import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    '''
    Launches the waypoint following mission stack, including localization,
    waypoint following, controls, and sensor nodes (if not in sim mode).
    Removes MOOS dependencies.
    '''

    # --- Parameter Parsing ---
    sim = "false"
    verbose = "false"
    param_file = '/home/frostlab/config/vehicle_params.yaml'
    namespace = '' # Initialize namespace to empty, can be set via arg

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]
        elif arg.startswith('param_file:='):
            param_file = arg.split(':=')[1]
        elif arg.startswith("sim:="):
            sim = arg.split(":=")[1].lower()
        elif arg.startswith("verbose:="):
            verbose = arg.split(":=")[1].lower()

    output_config = 'screen' if verbose == "true" else 'log'

    # --- Package Directories ---
    localization_pkg_dir = get_package_share_directory('cougars_localization')
    control_pkg_dir = get_package_share_directory('cougars_control')

    # --- Mission File Path ---
    mission_yaml_path = os.path.join(
        control_pkg_dir,
        'waypoint_missions',
        'provo_marina_mission.yaml'
    )
    
    # --- Launch Actions List ---
    launch_actions = []

    # --- Include Sensor Launch (if not sim) ---
    if sim == "false":
        sensors_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(localization_pkg_dir, 'launch', "sensors_launch.py")
            )
            # You might need to pass parameters like 'namespace' or 'param_file'
            # to included launch files if they support it.
        )
        launch_actions.append(sensors_launch)

    # --- Include Converters Launch ---
    converters_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg_dir, 'launch', "converters_launch.py")
        )
        # Add parameters if needed
    )
    launch_actions.append(converters_launch)

    # --- Define Nodes ---
    waypoint_follower_node = Node(
        package='cougars_control',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace=namespace,
        output='screen', # Keep waypoint follower on screen for visibility
        parameters=[
            {'waypoint_file_path': mission_yaml_path},
            {'slip_radius': 3.0}, # meters
            {'desired_travel_speed': 25.0}, # Example speed
            {'loop_rate': 5.0} # Hz
        ]
    )

    coug_controls_node = Node(
        package='cougars_control',
        executable='coug_controls',
        namespace=namespace,
        parameters=[param_file],
        output=output_config,
    )

    coug_kinematics_node = Node(
        package='cougars_control',
        executable='coug_kinematics',
        namespace=namespace,
        parameters=[param_file],
        output=output_config,
    )

    factor_graph_node = Node(
        package='cougars_localization',
        executable='factor_graph.py',
        namespace=namespace,
        parameters=[param_file],
        output=output_config,
    )

    modem_pinger_node = Node(
        package='seatrac',
        executable='modem_pinger',
        namespace=namespace,
        parameters=[param_file],
        output=output_config,
    )

    rf_bridge_node = Node(
        package='cougars_coms',
        executable='rf_bridge.py',
        namespace=namespace,
        parameters=[param_file],
        output=output_config,
    )

    emergency_protocols_node = Node(
            package='cougars_control',
            executable='emergency_protocols',
            namespace=namespace,
            parameters=[param_file],
            output=output_config, # Or 'screen' if you always want to see it
    )

    # --- Add Nodes to Launch Actions ---
    launch_actions.extend([
        waypoint_follower_node,
        coug_controls_node,
        coug_kinematics_node,
        factor_graph_node,
        modem_pinger_node,
        rf_bridge_node,
        emergency_protocols_node, # Added for safety
    ])

    # --- Return Launch Description ---
    return LaunchDescription(launch_actions)