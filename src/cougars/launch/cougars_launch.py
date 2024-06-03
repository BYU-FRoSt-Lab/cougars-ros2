import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen'),
        launch_ros.actions.Node(
            package='cougars',
            executable='controller'),
        launch_ros.actions.Node(
            package='cougars',
            executable='leak_sub'),
        launch_ros.actions.Node(
            package='cougars',
            executable='volt_sub'),
        launch_ros.actions.Node(
            package='seatrac',
            executable='modem'),
  ])