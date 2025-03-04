import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class RFBridge(Node):
    def __init__(self):
        super().__init__('rf_bridge')
        
        # Serial configuration
        self.ser = serial.Serial(
            port='/dev/ttyAMA0',
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.1
        )
        
        # Create publisher for received RF messages
        self.publisher = self.create_publisher(String, 'rf_received', 10)
        
        # Create subscriber for outgoing RF messages
        self.subscription = self.create_subscription(
            String,
            'rf_transmit',
            self.tx_callback,
            10)
        
        # Start serial read thread
        self.serial_thread = threading.Thread(target=self.serial_read_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info("RF Bridge node started")

    def tx_callback(self, msg):
        """Handle incoming ROS messages for transmission"""
        try:
            message = msg.data + '\n'
            self.ser.write(message.encode())
            self.get_logger().debug(f"Sent via RF: {message.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write error: {str(e)}")

    def serial_read_loop(self):
        """Continuous serial read loop"""
        buffer = ''
        while rclpy.ok():
            try:
                data = self.ser.read(self.ser.in_waiting or 1).decode()
                if data:
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith('SEND:'):  # Trigger string
                            msg = String()
                            msg.data = line[5:]  # Remove 'SEND:' prefix
                            self.publisher.publish(msg)
                            self.get_logger().debug(f"Received via RF: {line}")
            except Exception as e:
                self.get_logger().error(f"Serial read error: {str(e)}")
                break

def main(args=None):
    rclpy.init(args=args)
    rf_bridge = RFBridge()
    try:
        rclpy.spin(rf_bridge)
    except KeyboardInterrupt:
        rf_bridge.get_logger().info("Shutting down RF Bridge node")
    finally:
        rf_bridge.ser.close()
        rf_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
