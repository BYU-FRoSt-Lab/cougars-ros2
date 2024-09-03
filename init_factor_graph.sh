# sends an empty msg to factor_graph.py - this is the OK to begin factor graph

# should be after checking for DVL-lock and restarting DVL

ros2 topic pub /init std_msgs/msg/Empty -1