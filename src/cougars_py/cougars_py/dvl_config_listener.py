import rclpy
from rclpy.node import Node
from dvl_msgs.msg import ConfigStatus
from rclpy.qos import QoSProfile, HistoryPolicy

class DvlCommandResponseListener(Node):

    def __init__(self):
        super().__init__('dvl_command_response_listener')

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create a subscriber to the 'dvl/command/response' topic
        self.subscription = self.create_subscription(
            ConfigStatus,
            'dvl/config/status',
            self.response_callback,
            qos_profile  # QoS profile set to queue up to 10 messages
        )
        self.subscription  # Prevent unused variable warning

    def response_callback(self, msg):
        self.get_logger().info("status update")
        # Log the received message contents
        self.get_logger().info(f"Received CommandResponse: success = {msg.success}, message = '{msg.message}'")

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    listener = DvlCommandResponseListener()

    # Spin the node so it keeps running
    rclpy.spin(listener)

    # Shutdown the node when done
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
