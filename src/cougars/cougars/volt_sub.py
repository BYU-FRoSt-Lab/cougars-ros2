import rclpy
from rclpy.node import Node
from frost_interfaces.srv import EmergencyStop
from frost_interfaces.msg import Volt
from rclpy.qos import qos_profile_sensor_data

CRITICAL_VOLTAGE = 14.0


class VoltageSubscriber(Node):
    def __init__(self):
        super().__init__("voltage_subscriber")
        self.subscription = self.create_subscription(
            Volt, "volt_data", self.listener_callback, qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning
        self.cli = self.create_client(EmergencyStop, "emergency_stop")
        while not self.cli.wait_for_service(timeout_sec=1):
            self.get_logger().info("EmergencyStop service not available, waiting...")
        self.req = EmergencyStop.Request()

    def send_request(self, err):
        self.req.error = err
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def listener_callback(self, msg):
        if msg.voltage < CRITICAL_VOLTAGE:
            error = "ERROR: Low Voltage (" + str(msg.voltage) + ")"
            self.send_request(error)


def main(args=None):
    rclpy.init(args=args)

    voltage_subscriber = VoltageSubscriber()

    rclpy.spin(voltage_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    voltage_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()