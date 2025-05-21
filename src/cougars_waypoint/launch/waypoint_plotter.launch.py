import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'cougars_waypoint' # CHANGED
    pkg_share_dir = get_package_share_directory(pkg_name) # CHANGED
    mapviz_config_file = os.path.join(pkg_share_dir, 'config', 'mapviz_plotter_params.mvc')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    mapviz_node = launch_ros.actions.Node(
        package='mapviz',
        executable='mapviz',
        name='mapviz',
        parameters=[
            {'config': mapviz_config_file},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    initialize_origin_node = launch_ros.actions.Node(
        package='swri_transform_util',
        executable='initialize_origin.py',
        name='initialize_origin',
        remappings=[
            ('fix', '/mapviz/origin')
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    static_tf_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_origin_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'origin'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    map_point_converter_node = launch_ros.actions.Node(
        package=pkg_name, # CHANGED
        executable='map_point_converter_node',
        name='map_point_converter_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    waypoint_saver_node = launch_ros.actions.Node(
        package=pkg_name, # CHANGED
        executable='waypoint_saver_node',
        name='waypoint_saver_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )

    return launch.LaunchDescription([
        declare_use_sim_time_cmd,
        mapviz_node,
        initialize_origin_node,
        static_tf_node,
        map_point_converter_node,
        waypoint_saver_node,
        launch.actions.LogInfo(msg=f"""
        Cougars Waypoint Tool Ready! (Package: {pkg_name})
        1. (If not done by another node) Publish an initial NavSatFix message to /mapviz/origin to set the map's geographic origin.
           Example: ros2 topic pub --once /mapviz/origin sensor_msgs/NavSatFix "{{latitude: 40.0, longitude: -105.0, altitude: 1500.0}}"
        2. In Mapviz, use the 'Waypoint Clicker' tool (top toolbar, usually a crosshair icon) to select points on the map.
        3. These points will be converted to NavSatFix messages on /mapviz/goal and displayed.
        4. To save collected waypoints, call the /save_waypoints service.
           Example: ros2 service call /save_waypoints std_srvs/srv/Trigger "{{}}"
        """)
    ])