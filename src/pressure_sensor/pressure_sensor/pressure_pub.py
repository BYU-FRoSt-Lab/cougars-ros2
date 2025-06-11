import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from . import ms5837

class PressurePublisher(Node):
    def __init__(self):
        super().__init__('pressure_publisher')
        self.declare_parameter('sensor_type', 0)  # 0: 02BA, 1: 30BA ##default to 02BA
        sensor_type = self.get_parameter('sensor_type').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(FluidPressure, 'pressure/data', 10)

        if sensor_type == 0:
            self.sensor = ms5837.MS5837_02BA()
            self.get_logger().info("Using MS5837_02BA sensor (shallow)")
        else:
            self.sensor = ms5837.MS5837_30BA()
            self.get_logger().info("Using MS5837_30BA sensor (deep)")

        if not self.sensor.init():
            self.get_logger().error("Sensor could not be initialized")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(0.1, self.timer_callback)  # 1 Hz

    def timer_callback(self):
        if self.sensor.read():
            pressure_pa = self.sensor.pressure() * 100  # mbar to Pascal
            msg = FluidPressure()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.fluid_pressure = pressure_pa
            msg.variance = 0.0

            self.publisher_.publish(msg)
            # self.get_logger().info(f"Published pressure: {pressure_pa:.2f} Pa")
        else:
            self.get_logger().error("Sensor read failed!")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = PressurePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
