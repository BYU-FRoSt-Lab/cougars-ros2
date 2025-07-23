import sys

import launch
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import yaml

def generate_launch_description():
    '''
    :author: Nelson Durrant
    :date: September 2024
    
    Launches the sensor nodes for the vehicle.

    :return: The launch description.
    '''

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='coug0'
    )
    sim_launch_arg = DeclareLaunchArgument(
        'sim',
        default_value='False'
    )
    param_file_launch_arg = DeclareLaunchArgument(
        'param_file',
        default_value='/home/frostlab/config/deploy_tmp/vehicle_params.yaml'
    )
    fleet_param_launch_arg = DeclareLaunchArgument(
        'fleet_param',
        default_value='/home/frostlab/config/deploy_tmp/fleet_params.yaml'
    )
    
    return launch.LaunchDescription([
        
        # Serial Teensy connection
        launch_ros.actions.Node(
            package='fin_sub_cpp', 
            executable='control_node', 
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param'),
                        {'demo_mode': LaunchConfiguration('sim')}],
            output='log',
        ),
        # Start the control nodes
        launch_ros.actions.Node(
            package='cougars_control',
            executable='coug_kinematics',
            parameters=[LaunchConfiguration('param_file'), LaunchConfiguration('fleet_param')],
            namespace=LaunchConfiguration('namespace'),
        ),
    ])