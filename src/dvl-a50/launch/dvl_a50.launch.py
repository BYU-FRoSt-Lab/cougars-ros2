import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    launch.actions.DeclareLaunchArgument('ip_address', default_value='192.168.2.95'),
    ip_address = '192.168.2.95'
    dvl_a50 = launch_ros.actions.Node(
        package='dvl_a50', 
        executable='dvl_a50_sensor', 
        parameters=[{'dvl_ip_address': ip_address}],
        output='screen')


    return launch.LaunchDescription([
        dvl_a50
    ])


       
