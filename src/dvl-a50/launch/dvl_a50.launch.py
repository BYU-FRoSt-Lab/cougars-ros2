import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import sys


def generate_launch_description():
    # NOTE this line below does not work but maybe we could see about implmenting it if we need to
    launch.actions.DeclareLaunchArgument('ip_address', default_value='192.168.2.95'),
    
    param_file = '/home/frostlab/config/vehicle_params.yaml'
    namespace = ''

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]
        if arg.startswith('param_file:='):
            param_file = arg.split(':=')[1]
        if arg.startswith("verbose:="):
            verbose = arg.split(":=")[1].lower()

    dvl_a50 = launch_ros.actions.Node(
        package='dvl_a50', 
        executable='dvl_a50_sensor', 
        parameters=[param_file],
        namespace=namespace,
        # parameters=[{'dvl_ip_address': ip_address}],
        output='screen')


    return launch.LaunchDescription([
        dvl_a50
    ])


       
