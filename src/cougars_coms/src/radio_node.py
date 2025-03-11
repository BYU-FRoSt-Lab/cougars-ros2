import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import TwistStamped, PoseStamped
import serial
import threading
import json

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
        
        # Create publisher for init commands
        self.init_publisher = self.create_publisher(String, '/coug1/init', 10)
        
        # Create subscriber for outgoing RF messages
        self.subscription = self.create_subscription(
            String,
            'rf_transmit',
            self.tx_callback,
            10)
        
        # Create a subscriber for the data to send when STATUS is received
        self.status_data_sub = self.create_subscription(
            String,
            'status_data',
            self.status_data_callback,
            10)
        
        # Create subscribers for all the requested status topics
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/coug1/odom', 
            self.odom_callback, 
            10)
            
        self.gps_odom_sub = self.create_subscription(
            Odometry, 
            '/coug1/gps_odom', 
            self.gps_odom_callback, 
            10)
            
        self.leak_sub = self.create_subscription(
            Bool, 
            '/coug1/leak/data', 
            self.leak_callback, 
            10)
            
        self.battery_sub = self.create_subscription(
            BatteryState, 
            '/coug1/battery/data', 
            self.battery_callback, 
            10)
            
        self.dvl_velocity_sub = self.create_subscription(
            TwistStamped, 
            '/coug1/dvl/velocity', 
            self.dvl_velocity_callback, 
            10)
            
        self.dvl_position_sub = self.create_subscription(
            PoseStamped, 
            '/coug1/dvl/position', 
            self.dvl_position_callback, 
            10)
        
        # Variables to store the latest data
        self.latest_status_data = "NO_DATA"
        self.latest_odom = "NO_DATA"
        self.latest_gps_odom = "NO_DATA"
        self.latest_leak = "NO_DATA"
        self.latest_battery = "NO_DATA"
        self.latest_dvl_velocity = "NO_DATA"
        self.latest_dvl_position = "NO_DATA"
        
        # Start serial read thread
        self.serial_thread = threading.Thread(target=self.serial_read_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info("RF Bridge node started")

    def status_data_callback(self, msg):
        """Store the latest data to send when STATUS is received"""
        self.latest_status_data = msg.data
        self.get_logger().debug(f"Updated status data: {self.latest_status_data}")

    def odom_callback(self, msg):
        """Store latest odometry data as a simplified string"""
        pos = msg.pose.pose.position
        self.latest_odom = f"odom:x={pos.x:.2f},y={pos.y:.2f},z={pos.z:.2f}"
        self.get_logger().debug(f"Updated odom data")
        
    def gps_odom_callback(self, msg):
        """Store latest GPS odometry data as a simplified string"""
        pos = msg.pose.pose.position
        self.latest_gps_odom = f"gps:x={pos.x:.2f},y={pos.y:.2f},z={pos.z:.2f}"
        self.get_logger().debug(f"Updated GPS odom data")
        
    def leak_callback(self, msg):
        """Store latest leak status"""
        self.latest_leak = f"leak:{msg.data}"
        self.get_logger().debug(f"Updated leak data: {self.latest_leak}")
        
    def battery_callback(self, msg):
        """Store latest battery information"""
        self.latest_battery = f"batt:{msg.percentage:.1f}%,{msg.voltage:.1f}V"
        self.get_logger().debug(f"Updated battery data")
        
    def dvl_velocity_callback(self, msg):
        """Store latest DVL velocity data"""
        twist = msg.twist
        self.latest_dvl_velocity = f"dvl_v:x={twist.linear.x:.2f},y={twist.linear.y:.2f},z={twist.linear.z:.2f}"
        self.get_logger().debug(f"Updated DVL velocity data")
        
    def dvl_position_callback(self, msg):
        """Store latest DVL position data"""
        pos = msg.pose.position
        self.latest_dvl_position = f"dvl_p:x={pos.x:.2f},y={pos.y:.2f},z={pos.z:.2f}"
        self.get_logger().debug(f"Updated DVL position data")

    def tx_callback(self, msg):
        """Handle incoming ROS messages for transmission"""
        try:
            message = msg.data + '\n'
            self.ser.write(message.encode())
            self.get_logger().debug(f"Sent via RF: {message.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write error: {str(e)}")
    
    def get_all_status_data(self):
        """Compile all status data into a single string"""
        status_data = {
            "odom": self.latest_odom,
            "gps": self.latest_gps_odom,
            "leak": self.latest_leak,
            "battery": self.latest_battery,
            "dvl_vel": self.latest_dvl_velocity,
            "dvl_pos": self.latest_dvl_position,
        }
        return json.dumps(status_data)

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
                        
                        if line == 'STATUS':  # When STATUS is received
                            # Send all the latest data over the serial line
                            response = self.get_all_status_data() + '\n'
                            self.ser.write(response.encode())
                            self.get_logger().info(f"Received STATUS, responding with sensor data")
                            self.get_logger().debug(f"Status response: {response.strip()}")
                            
                        elif line == 'INIT':  # When INIT is received
                            # Publish to the init topic
                            init_msg = String()
                            init_msg.data = "INIT_COMMAND"
                            self.init_publisher.publish(init_msg)
                            self.get_logger().info(f"Received INIT, published to /coug1/init topic")
                            
                            # Also send confirmation back over RF
                            response = "INIT_ACK\n"
                            self.ser.write(response.encode())
                            
                        else:
                            # For other messages, publish them as before
                            msg = String()
                            msg.data = line
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