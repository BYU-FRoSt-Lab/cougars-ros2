import sys

import launch
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    '''
    :date: March 31
    
    Launches the sensor nodes for the vehicle.

    :return: The launch description.
    '''

    param_file = '/home/frostlab/config/vehicle_params.yaml'
    verbose = "false"
    namespace=''

    for arg in sys.argv:
        if arg.startswith('namespace:='):
            namespace = arg.split(':=')[1]
        if arg.startswith('param_file:='):
            param_file = arg.split(':=')[1]
        if arg.startswith("verbose:="):
            verbose = arg.split(":=")[1].lower()
    
    if verbose == "true":
        output = 'screen'
    else:
        output = 'log'

    launch_actions = []

    
    launch_actions.extend([
        
        launch_ros.actions.Node(
            package='cougars_bringup',
            executable='bag_recorder',
            # parameters=[param_file], #TODO 
            namespace=namespace,
            output=output,
        ),
    ])

    return launch.LaunchDescription(launch_actions)