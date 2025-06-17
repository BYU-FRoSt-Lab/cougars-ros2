import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from . import ms5837

class PressurePublisher(Node):
    def __init__(self, sensor_model, node_name):
        super().__init__(node_name)
        self.declare_parameter('sensor_type', sensor_model)
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
            return

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

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
    node = None
    node2 = None
    try:
        node = PressurePublisher(0, 'pressure_publisher_02ba')
        node2 = PressurePublisher(1, 'pressure_publisher_30ba')
        if node is None or node2 is None:
            rclpy.shutdown()
            return
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.add_node(node2)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if node2 is not None:
            node2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
