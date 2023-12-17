import rclpy
from rclpy.node import Node
from frost_interfaces.srv import EmergencyStop
from frost_interfaces.msg import Humid
from rclpy.qos import qos_profile_sensor_data

SERVICE_TIMEOUT = 1  # seconds


class HumiditySubscriber(Node):
    def __init__(self):
        super().__init__("humidity_subscriber")
        self.subscription = self.create_subscription(
            Humid, "humidity", self.listener_callback, qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning
        self.cli = self.create_client(EmergencyStop, "emergency_stop")
        while not self.cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().info("EmergencyStop service not available, waiting...")
        self.req = EmergencyStop.Request()

    def send_request(self, err):
        self.req.error = err
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def listener_callback(self, msg):
        error = "ERROR: High Humidity (" + str(msg.humidity) + ")"
        self.send_request(error)


def main(args=None):
    rclpy.init(args=args)

    humidity_subscriber = HumiditySubscriber()

    rclpy.spin(humidity_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    humidity_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()