import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', '~/ros2_ws/rosbag/'],
            output='screen'),
        launch_ros.actions.Node(
            package='frost_uuv',
            executable='controller'),
        launch_ros.actions.Node(
            package='frost_uuv',
            executable='leak_sub'),
        launch_ros.actions.Node(
            package='frost_uuv',
            executable='voltage_sub'),
        launch_ros.actions.Node(
            package='frost_uuv',
            executable='humidity_sub'),
        # launch_ros.actions.Node(
        #   package='seatrac',
        #   executable='modem_pub'),
  ])