
# Static Transform Publisher
# Creates a static transforms for important coordinate frames
# the first 3 numbers are translation (x,y,z)
# the second 4 numbers corespond to the quaternion (qx, qy, qz, qw)
# thus all values in order are x y z qx qy qz qw
# following that we have the initial coordinate frame followed by the target one


ros2 run tf2_ros static_transform_publisher 0 0 0 0.7071 0.7071 0 0 enu ned &

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 modem robot &
