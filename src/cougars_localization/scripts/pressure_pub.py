#!/usr/bin/python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
# from . import ms5837
import ms5837

class PressurePublisher(Node):
    def __init__(self):
        super().__init__('pressure_pub')
        
        self.declare_parameter('sensor_type', 0)
        sensor_type = self.get_parameter('sensor_type').get_parameter_value().integer_value


        self.declare_parameter('i2c_bus', 1)
        i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value

        self.declare_parameter('frame_id', 'pressure_link')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(FluidPressure, 'pressure/data', 10)

        if sensor_type == 0:
            self.sensor = ms5837.MS5837_02BA(bus=i2c_bus)
            self.get_logger().info("Using MS5837_02BA sensor (shallow)")
        else:
            self.sensor = ms5837.MS5837_30BA(bus=i2c_bus)
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
            msg.header.frame_id = self.frame_id
            msg.fluid_pressure = pressure_pa
            msg.variance = 0.0

            self.publisher_.publish(msg)
            # self.get_logger().info(f"Published pressure: {pressure_pa:.2f} Pa")
        else:
            self.get_logger().error("Sensor read failed!")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PressurePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
