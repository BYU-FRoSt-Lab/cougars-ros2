import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from frost_interfaces.msg import SystemStatus
from dvl_msgs.msg import ConfigCommand  # Replace with correct package

class DVLAcousticController(Node):
    def __init__(self):
        super().__init__('dvl_acoustic_controller')

        # QoS to match the publisher
        qos_reliable_profile = QoSProfile(depth=5)
        qos_reliable_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_reliable_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Publisher to DVL config command
        self.publisher = self.create_publisher(ConfigCommand, 'dvl/config/command', 10)

        # Subscriber with matching QoS
        self.subscription = self.create_subscription(
            SystemStatus,
            'system/status',
            self.listener_callback,
            qos_reliable_profile
        )

    def listener_callback(self, msg: SystemStatus):
        cmd_msg = ConfigCommand()
        cmd_msg.command = "set_config"
        cmd_msg.parameter_name = "acoustic_enabled"
        cmd_msg.parameter_value = "true" if msg.dvl_acoustics.data else "false"

        self.get_logger().warn(f"Setting acoustic_enabled to {cmd_msg.parameter_value}")
        self.publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DVLAcousticController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
