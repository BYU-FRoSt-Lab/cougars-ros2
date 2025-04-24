#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, FluidPressure
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from dvl_msgs.msg import DVLDR
import serial
import threading
import json
import queue
import traceback


class RFBridge(Node):
    def __init__(self):
        super().__init__('rf_bridge')
        
        # Flag to control thread execution
        self.running = True
        
        # Queue for communication between threads
        self.data_queue = queue.Queue()

    
        # Add debug parameter
        self.debug_mode = self.declare_parameter('debug_mode', False).value

        if self.debug_mode:
            self.get_logger().info("Debug mode enabled: Will log detailed packet information")

        # Create QoS profile with BEST_EFFORT for odometry
        self.odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # Number of samples to keep
        )

        # Create QoS profile for DVL data
        self.dvl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Variables to store the latest data
        self.latest_status_data = "NO_DATA"
        self.latest_odom = "NO_DATA"
        self.latest_leak = "NO_DATA"
        self.latest_battery = "NO_DATA"
        self.latest_dvl_velocity = "NO_DATA"
        self.latest_dvl_position = "NO_DATA"

        # Serial configuration
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB1',
                baudrate=9600,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.1
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")
            raise e
        
        # Create publisher for received RF messages
        self.publisher = self.create_publisher(String, 'rf_received', 10)
        
        # Create publisher for init commands
        self.init_publisher = self.create_publisher(String, 'init', 10)
        
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
            'odom', 
            self.odom_callback, 
            qos_profile=self.odom_qos)
            
        self.leak_sub = self.create_subscription(
            FluidPressure, 
            'leak/data', 
            self.leak_callback, 
            10)
            
        self.battery_sub = self.create_subscription(
            BatteryState, 
            'battery/data', 
            self.battery_callback, 
            10)
            
        self.dvl_velocity_sub = self.create_subscription(
            TwistWithCovarianceStamped, 
            'dvl/velocity', 
            self.dvl_velocity_callback, 
            qos_profile=self.dvl_qos)
            
        self.dvl_position_sub = self.create_subscription(
            DVLDR, 
            'dvl/position', 
            self.dvl_position_callback, 
            qos_profile=self.dvl_qos)
        

        
        # Start serial read thread
        self.serial_thread = threading.Thread(target=self.serial_read_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        # Start data processing thread
        self.process_thread = threading.Thread(target=self.process_data_loop)
        self.process_thread.daemon = True
        self.process_thread.start()
        
        # Create timer for queue monitoring (prints queue stats)
        self.create_timer(1.0, self.queue_monitor)
        
        self.get_logger().info("RF Bridge node started with queue-based threading")

    def status_data_callback(self, msg):
        """Store the latest data to send when STATUS is received"""
        # Make sure this is only handling String messages
        if hasattr(msg, 'data'):
            self.latest_status_data = msg.data
            self.get_logger().debug(f"Updated status data: {self.latest_status_data}")
        else:
            self.get_logger().error(f"Received non-String message in status_data_callback")

    def odom_callback(self, msg):
        """Store latest odometry data"""
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        self.latest_odom = f"odom:x={pos.x:.2f},y={pos.y:.2f},z={pos.z:.2f},vx={vel.x:.2f},vy={vel.y:.2f}"
        self.get_logger().debug("Updated odom data")
        
    def leak_callback(self, msg):
        """Store latest leak data"""
        if hasattr(msg, 'fluid_pressure'):
            self.latest_leak = f"leak:{msg.fluid_pressure:.2f}"
            self.get_logger().debug(f"Updated leak data: {self.latest_leak}")
        else:
            self.get_logger().error("Received message without fluid_pressure field in leak_callback")

    def battery_callback(self, msg):
        """Store latest battery data"""
        self.latest_battery = f"batt:v={msg.voltage:.1f},pct={msg.percentage*100:.0f}"
        self.get_logger().debug("Updated battery data")
            
    def dvl_velocity_callback(self, msg):
        """Store latest DVL velocity data"""
        lin = msg.twist.twist.linear
        self.latest_dvl_velocity = f"dvl_v:x={lin.x:.2f},y={lin.y:.2f},z={lin.z:.2f}"
        self.get_logger().debug("Updated DVL velocity data")
        
    def dvl_position_callback(self, msg):
        """Store latest DVL position data"""
        pos = msg.position
        self.latest_dvl_position = f"dvl_p:x={pos.x:.2f},y={pos.y:.2f},z={pos.z:.2f},r={msg.roll:.2f},p={msg.pitch:.2f},y={msg.yaw:.2f}"
        self.get_logger().debug("Updated DVL position data")

    def tx_callback(self, msg):
        """Handle incoming ROS messages for transmission via XBee"""
        try:
            message = msg.data
            self.send_xbee_data(message)
            self.get_logger().debug(f"Sent via XBee: {message}")
        except Exception as e:
            self.get_logger().error(f"XBee transmission error: {str(e)}")
            self.get_logger().error(traceback.format_exc())
    
    def get_all_status_data(self):
        """Get all status data in a compact format"""
        data_dict = {
            "odom": self.latest_odom,
            "leak": self.latest_leak,
            "battery": self.latest_battery,
            "dvl_vel": self.latest_dvl_velocity,
            "dvl_pos": self.latest_dvl_position,
        }
        
        # Remove empty or NO_DATA fields
        data_dict = {k: v for k, v in data_dict.items() if v and v != "NO_DATA"}
        
        return json.dumps(data_dict, separators=(',', ':'))

    
    def queue_monitor(self):
        """Monitor queue status"""
        self.get_logger().info(f"Queue size: {self.data_queue.qsize()}")
    
    def serial_read_loop(self):
        """Thread that only reads from serial and adds to queue"""
        buffer = b''
        FRAME_START = 0x7E  # XBee frame start delimiter
        
        while self.running and rclpy.ok():
            try:
                # Read raw bytes without immediate decoding
                raw_data = self.ser.read(self.ser.in_waiting or 1)
                
                if raw_data:
                    # Print raw bytes for debugging
                    hex_data = ' '.join([f'{b:02x}' for b in raw_data])
                    self.get_logger().debug(f"Raw bytes: {hex_data}")
                    
                    # Append to buffer
                    buffer += raw_data
                    
                    # Process complete API frames
                    while len(buffer) > 3:  # Need at least start byte + length bytes
                        # Find start of frame
                        start_idx = buffer.find(bytes([FRAME_START]))
                        if start_idx == -1:
                            buffer = b''  # Clear buffer if no start byte
                            break
                        elif start_idx > 0:
                            buffer = buffer[start_idx:]  # Discard bytes before start
                        
                        # Check if we have enough bytes for length
                        if len(buffer) < 3:
                            break  # Wait for more data
                        
                        # Get frame length (MSB, LSB format)
                        length = (buffer[1] << 8) | buffer[2]
                        total_length = length + 4  # Start + 2 length bytes + payload + checksum
                        
                        # Check if complete frame is available
                        if len(buffer) < total_length:
                            break  # Wait for more data
                        
                        # Extract the frame
                        frame = buffer[:total_length]
                        buffer = buffer[total_length:]  # Remove processed frame
                        
                        # Process the API frame based on frame type
                        if len(frame) > 3 and frame[3] == 0x90:  # ZigBee Receive Packet
                            # Extract payload data - starts at position 15 in Zigbee Rx frames
                            if len(frame) > 15:
                                payload = frame[15:-1]  # Exclude checksum
                                try:
                                    command = payload.decode('ascii')
                                    self.get_logger().info(f"Decoded command: {command}")
                                    
                                    # Put in processing queue
                                    self.data_queue.put(("command", command))
                                except UnicodeDecodeError:
                                    self.get_logger().warning(f"Failed to decode payload")
            except Exception as e:
                self.get_logger().error(f"Serial read error: {str(e)}")


    def process_data_loop(self):
        """Thread that processes data from the queue"""
        while self.running and rclpy.ok():
            try:
                # Get item from queue with timeout
                try:
                    item_type, data = self.data_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                
                if item_type == "line":
                    # Original processing for line-based inputs
                    line_bytes = data
                    # [existing code for processing lines]
                    
                elif item_type == "command":
                    # Direct command processing (already decoded)
                    command = data
                    
                    if command == "STATUS":
                        # Send all the latest data over the serial line using XBee API format
                        response = self.get_all_status_data()
                        self.send_xbee_data(response)
                        self.get_logger().info(f"Received STATUS, responding with sensor data")
                        self.get_logger().debug(f"Status response: {response}")
                        
                    elif command == "INIT":
                        # Publish to the init topic
                        init_msg = String()
                        init_msg.data = "INIT_COMMAND"
                        self.init_publisher.publish(init_msg)
                        self.get_logger().info(f"Received INIT, published to init topic")
                        
                        # Send confirmation back over RF using XBee API format
                        self.send_xbee_data("INIT_ACK")
                
                # Mark task as done
                self.data_queue.task_done()
                
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Process data error: {str(e)}")
                    self.get_logger().error(traceback.format_exc())
                
        self.get_logger().info("Process data thread ending")

    def send_xbee_data(self, message):
        """Construct and send an XBee API frame for broadcast transmission"""
        # Start delimiter
        frame = bytearray([0x7E])
        
        # Payload data
        payload = message.encode('ascii')
        
        # Frame type (0x10 = Zigbee Transmit Request)
        api_frame = bytearray([0x10])
        
        # Frame ID (use 0x01 to get a status response)
        api_frame.append(0x01)
        
        # 64-bit destination address (0xFFFF = broadcast)
        api_frame.extend([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF])
        
        # 16-bit destination network address (0xFFFE = broadcast)
        api_frame.extend([0xFF, 0xFE])
        
        # Broadcast radius (0 = max hops)
        api_frame.append(0x00)
        
        # Options (0 = default)
        api_frame.append(0x00)
        
        # Add the payload
        api_frame.extend(payload)
        
        # Calculate length (API frame data length)
        length = len(api_frame)
        frame.append((length >> 8) & 0xFF)  # MSB
        frame.append(length & 0xFF)         # LSB
        
        # Add the API frame
        frame.extend(api_frame)
        
        # Calculate checksum
        checksum = 0
        for b in api_frame:
            checksum += b
        checksum = 0xFF - (checksum & 0xFF)
        frame.append(checksum)
        
        # Send the frame
        self.ser.write(frame)
        
        # Debug
        hex_data = ' '.join([f'{b:02x}' for b in frame])
        self.get_logger().debug(f"Sent XBee frame: {hex_data}")
        self.get_logger().info(f"Sent message: {message}")


    def cleanup(self):
        """Clean up resources"""
        self.running = False
        
        # Wait for threads to finish
        if hasattr(self, 'serial_thread') and self.serial_thread.is_alive():
            self.serial_thread.join(1.0)
            
        if hasattr(self, 'process_thread') and self.process_thread.is_alive():
            self.process_thread.join(1.0)
            
        # Close serial port
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        rf_bridge = RFBridge()
        
        try:
            rclpy.spin(rf_bridge)
        except KeyboardInterrupt:
            rf_bridge.get_logger().info("Shutting down RF Bridge node")
        finally:
            rf_bridge.cleanup()
            rf_bridge.destroy_node()
            
    except Exception as e:
        print(f"Error initializing node: {str(e)}")
        
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


