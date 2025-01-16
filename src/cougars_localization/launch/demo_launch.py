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

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]
        if arg.startswith('param_file:='):
            param_file = arg.split(':=')[1]

    with open(param_file, 'r') as f:
        vehicle_params = yaml.safe_load(f)
    
    return launch.LaunchDescription([
        
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
        # Start the control nodes
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_kinematics',
            parameters=[param_file],
            namespace=namespace,
        ),
    ])