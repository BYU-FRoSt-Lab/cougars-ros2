import rclpy
from rclpy.node import Node
from dvl_msgs.msg import CommandResponse

class DvlCommandResponseListener(Node):

    def __init__(self):
        super().__init__('dvl_command_response_listener')
        
        # Create a subscriber to the 'dvl/command/response' topic
        self.subscription = self.create_subscription(
            CommandResponse,
            'dvl/command/response',
            self.response_callback,
            10  # QoS profile set to queue up to 10 messages
        )
        self.subscription  # Prevent unused variable warning

    def response_callback(self, msg):
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
