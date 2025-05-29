import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from . import ms5837

class PressurePublisher(Node):
    def __init__(self):
        super().__init__('pressure_publisher')
        self.publisher_ = self.create_publisher(FluidPressure, 'pressure_data', 10)

        self.sensor = ms5837.MS5837_02BA()
        if not self.sensor.init():
            self.get_logger().error("Sensor could not be initialized")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz

    def timer_callback(self):
        if self.sensor.read():
            pressure_pa = self.sensor.pressure() * 100  # mbar to Pascal
            msg = FluidPressure()
            msg.fluid_pressure = pressure_pa
            msg.variance = 0.0

            self.publisher_.publish(msg)
            self.get_logger().info(f"Published pressure: {pressure_pa:.2f} Pa")
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

