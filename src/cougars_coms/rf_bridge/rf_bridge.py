#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, FluidPressure
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from dvl_msgs.msg import DVLDR
from std_srvs.srv import SetBool
from frost_interfaces.msg import SystemControl, SystemStatus

from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
from digi.xbee.exception import TransmitException

import json
import threading
import traceback

class RFBridge(Node):
    def __init__(self):
        super().__init__('rf_bridge')

        # Namespace logic not needed if you use the launch file

        # vehicle_ns = ''
        # try:
        #     params = self.get_node_parameters_interface().get_parameter_overrides()
        #     vehicle_namespaces = [key.split('.')[0] for key in params.keys()
        #                           if key.startswith('coug') and not key.startswith('/**')]
        #     if vehicle_namespaces:
        #         vehicle_ns = vehicle_namespaces[0]
        #         self.get_logger().info(f"Found vehicle namespace from params: {vehicle_ns}")
        # except Exception as e:
        #     self.get_logger().debug(f"Couldn't get namespace from parameters: {str(e)}")
        # if not vehicle_ns:
        #     node_namespace = self.get_namespace()
        #     if node_namespace and node_namespace != '/':
        #         vehicle_ns = node_namespace.strip('/')
        #         # self.get_logger().info(f"Using parent namespace: {vehicle_ns}")
        # if not vehicle_ns:
        #     vehicle_ns = self.declare_parameter('namespace', '').value
        #     if vehicle_ns:
        #         self.get_logger().info(f"Using explicitly provided namespace: {vehicle_ns}")
        # self.namespace = vehicle_ns.strip('/')
        # if self.namespace and not self.namespace.endswith('/'):
        #     self.namespace += '/'
        # # self.get_logger().info(f"Final vehicle namespace: '{self.namespace}'")

        # Debug mode
        self.debug_mode = self.declare_parameter('debug_mode', False).value
        if self.debug_mode:
            self.get_logger().info("Debug mode enabled: Will log detailed packet information")

        # QoS profiles
        self.odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        self.dvl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)

        # Data storage
        self.latest_safety_status = "NO_DATA"
        self.latest_dvl_pos = "NO_DATA"
        self.latest_battery = "NO_DATA"
        self.latest_depth = "NO_DATA"
        self.latest_pressure = "NO_DATA"

        self.vehicle_id = self.declare_parameter('vehicle_ID', 0).value
        self.base_station_id = self.declare_parameter('base_station_id', 15).value

        # XBee configuration
        self.xbee_port = self.declare_parameter('xbee_port', '/dev/ttyUSB0').value
        self.xbee_baud = self.declare_parameter('xbee_baud', 9600).value
        self.device = XBeeDevice(self.xbee_port, self.xbee_baud)
        try:
            self.device.open()
            self.get_logger().info(f"Opened XBee device on {self.xbee_port} at {self.xbee_baud} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to open XBee device: {e}")
            raise

        # ROS publishers and subscribers
        self.publisher = self.create_publisher(String, 'rf_received', 10)
        self.init_publisher = self.create_publisher(SystemControl, 'system/status', 10)

        self.e_kill_client = self.create_client(SetBool, "arm_thruster")

        self.subscription = self.create_subscription(
            String,
            'rf_transmit',
            self.tx_callback,
            10)

        self.subscription = self.create_subscription(
            SystemStatus,
            'safety_status',
            self.safety_status_callback,
            10)

        self.battery_sub = self.create_subscription(
            BatteryState,
            'battery/data',
            self.battery_callback,
            10)
        
        self.dvl_sub = self.create_subscription(
            DVLDR,
            'dvl/position',
            self.dvl_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'depth_data',
            self.depth_callback,
            10)
        
        self.pressure_sub = self.create_subscription(
            FluidPressure,
            'pressure/data',
            self.pressure_callback,
            10)


        # Register XBee data receive callback
        self.device.add_data_received_callback(self.data_receive_callback)
        self.get_logger().info("RF Bridge node started using digi-xbee library.")

        # Thread-safe shutdown flag
        self.running = True

    def safe_numeric_convert(self, value, convert_func=float):
        """Safely convert ROS2 message fields to Python numeric types"""
        try:
            # Handle numpy/ROS2 integer types (int8, int16, etc.)
            if hasattr(value, 'item'):
                return convert_func(value.item())
            # Handle regular Python types
            else:
                return convert_func(value)
        except (ValueError, TypeError, AttributeError):
            return convert_func(0)  # Default fallback value

    def battery_callback(self, msg):
        self.latest_battery = {
            "voltage": self.safe_numeric_convert(msg.voltage, float),
            "percentage": self.safe_numeric_convert(msg.percentage, float)
        }
        self.get_logger().debug("Updated battery data")

    def safety_status_callback(self, msg):
        self.latest_safety_status = {
            "depth_status": self.safe_numeric_convert(msg.depth_status, int),
            "gps_status": self.safe_numeric_convert(msg.gps_status, int),
            "modem_status": self.safe_numeric_convert(msg.modem_status, int),
            "dvl_status": self.safe_numeric_convert(msg.dvl_status, int),
            "emergency_status": self.safe_numeric_convert(msg.emergency_status, int)
        }
        self.get_logger().debug("Updated safety status data")

    def dvl_callback(self, msg):
        self.latest_dvl_pos = {
            "x": self.safe_numeric_convert(msg.position.x, float),
            "y": self.safe_numeric_convert(msg.position.y, float),
            "z": self.safe_numeric_convert(msg.position.z, float),
            "roll": self.safe_numeric_convert(msg.roll, float),
            "pitch": self.safe_numeric_convert(msg.pitch, float),
            "yaw": self.safe_numeric_convert(msg.yaw, float)
        }
        self.get_logger().debug("Updated DVL position data")

    def depth_callback(self, msg):
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
            pose = msg.pose.pose
            self.latest_depth = {
                "depth": self.safe_numeric_convert(pose.position.z, float)
            }
            self.get_logger().debug("Updated depth data")

    def tx_callback(self, msg):
        try:
            message = msg.data
            self.device.send_data_broadcast(message)
            self.get_logger().debug(f"Sent via XBee: {message}")
        except Exception as e:
            self.get_logger().error(f"XBee transmission error: {str(e)}")
            self.get_logger().error(traceback.format_exc())

    def send_message(self, msg, address):
        try:
            remote_device = RemoteXBeeDevice(self.device, address)
            self.device.send_data(remote_device, msg)
            self.get_logger().info(f"Sent via XBee: {msg}")
        except TransmitException as e:
            self.get_logger().error(f"XBee transmission error - TransmitException: {e}")
            self.get_logger().error(traceback.format_exc())
        except Exception as e:
            self.get_logger().error(f"XBee transmission error - Exception: {str(e)}")
            self.get_logger().error(traceback.format_exc())

    def get_all_status_data(self):
        data_dict = {
            "src_id" : self.vehicle_id,
            "message" : "STATUS",
            "safety_status": self.latest_safety_status,
            "dvl_pos": self.latest_dvl_pos,
            "battery_state": self.latest_battery,
            "depth_data": self.latest_depth,
            "pressure_data": self.latest_pressure,
        }
        data_dict = {k: v for k, v in data_dict.items() if v and v != "NO_DATA"}
        return json.dumps(data_dict, separators=(',', ':'))

    def data_receive_callback(self, xbee_message):
        try:
            payload = xbee_message.data.decode('utf-8', errors='replace')
            return_address = xbee_message.remote_device.get_64bit_addr()
            self.get_logger().info(f"Received from {return_address}: {payload}")
            
            msg = String()
            msg.data = payload
            self.publisher.publish(msg)
            self.get_logger().info(f"{payload}")
            if payload == "STATUS":
                response = self.get_all_status_data()
                self.send_message(response, return_address)
                self.get_logger().debug(f"Received STATUS, responding with sensor data")
                self.get_logger().debug(f"Status response: {response}")
            elif payload == "PING":
                response = {"message": "PING", "src_id": self.vehicle_id}
                self.send_message(json.dumps(response), return_address)
                self.get_logger().info(f"Received PING, responding with PING")
            elif payload == "E_KILL":
                self.kill_thruster()
            elif payload == "INIT":
                self.init_vehicle(payload)
        except Exception as e:
            self.get_logger().error(f"Error in data_receive_callback: {e}")
            self.get_logger().error(traceback.format_exc())

    
    def init_vehicle(self, msg):
        init_msg = SystemControl()
        init_msg.start = msg.start
        init_msg.rosbag_flag = msg.rosbag_flag
        init_msg.rosbag_prefix = msg.rosbag_prefix
        init_msg.thruster_arm = msg.thruster_arm
        init_msg.dvl_acoustic = msg.dvl_acoustic
        self.init_publisher.publish(init_msg)
        self.get_logger().info(f"Received INIT, published to init topic")
        self.device.send_data_broadcast("INIT_ACK")

    
    def kill_thruster(self):
        self.get_logger().info("Received kill command from base station")
        request = SetBool.Request()
        request.data = False

        while not self.thruster_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the arm_thruster service. Exiting.")
                return
            self.get_logger().info("arm_thruster service not available, waiting again...")

        future = self.thruster_client.call_async(request)

        def callback(fut):
            try:
                response = fut.result()
                if response.success:
                    self.get_logger().info("Thruster has been deactivated.")
                else:
                    self.get_logger().error("Failed to deactivate thruster.")

                msg_dict = {
                    "src_id": self.vehicle_id,  
                    "message": "E_KILL",
                    "success": response.success
                }
                msg = json.dumps(msg_dict)
                self.send_message(msg)

            except Exception as e:
                self.get_logger().error(f"Error while trying to deactivate thruster: {str(e)}")

        future.add_done_callback(callback)


    def destroy_node(self):
        self.running = False
        if self.device is not None and self.device.is_open():
            self.device.close()
            self.get_logger().info("XBee device closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RFBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Shutting down RF Bridge node.")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
