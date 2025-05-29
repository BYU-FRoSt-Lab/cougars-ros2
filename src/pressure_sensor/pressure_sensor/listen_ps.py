import rclpy
from rclpy.node import Node

from sensor_msgs.msg import FluidPressure

class FluidPressureSubscriber(Node):
    def __init__(self):
        super().__init__('fluid_pressure_subscriber')
        self.subscription = self.create_subscription(
            FluidPressure,            # subscribe to FluidPressure messages
            'pressure_data',                  # topic name (use the same topic you publish to)
            self.listener_callback,   # callback function
            10)                       # queue size
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: FluidPressure):
        # Print the fluid pressure in Pascals and variance
        self.get_logger().info(f'Received pressure: {msg.fluid_pressure} Pa, variance: {msg.variance}')

def main(args=None):
    print("listening...")
    rclpy.init(args=args)

    fluid_pressure_subscriber = FluidPressureSubscriber()

    rclpy.spin(fluid_pressure_subscriber)

    fluid_pressure_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
