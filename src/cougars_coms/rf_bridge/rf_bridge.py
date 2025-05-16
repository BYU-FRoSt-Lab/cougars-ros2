#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, FluidPressure
from geometry_msgs.msg import TwistWithCovarianceStamped
from dvl_msgs.msg import DVLDR

from digi.xbee.devices import XBeeDevice

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
        self.latest_status_data = "NO_DATA"
        self.latest_odom = "NO_DATA"
        self.latest_leak = "NO_DATA"
        self.latest_battery = "NO_DATA"
        self.latest_dvl_velocity = "NO_DATA"
        self.latest_dvl_position = "NO_DATA"

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
        self.init_publisher = self.create_publisher(String, 'init', 10)

        self.subscription = self.create_subscription(
            String,
            'rf_transmit',
            self.tx_callback,
            10)

        self.status_data_sub = self.create_subscription(
            String,
            'status_data',
            self.status_data_callback,
            10)

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

        # Register XBee data receive callback
        self.device.add_data_received_callback(self.data_receive_callback)
        self.get_logger().info("RF Bridge node started using digi-xbee library.")

        # Thread-safe shutdown flag
        self.running = True

    def status_data_callback(self, msg):
        if hasattr(msg, 'data'):
            self.latest_status_data = msg.data
            self.get_logger().debug(f"Updated status data: {self.latest_status_data}")
        else:
            self.get_logger().error(f"Received non-String message in status_data_callback")

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        self.latest_odom = f"odom:x={pos.x:.2f},y={pos.y:.2f},z={pos.z:.2f},vx={vel.x:.2f},vy={vel.y:.2f}"
        self.get_logger().debug("Updated odom data")

    def leak_callback(self, msg):
        if hasattr(msg, 'fluid_pressure'):
            self.latest_leak = f"leak:{msg.fluid_pressure:.2f}"
            self.get_logger().debug(f"Updated leak data: {self.latest_leak}")
        else:
            self.get_logger().error("Received message without fluid_pressure field in leak_callback")

    def battery_callback(self, msg):
        self.latest_battery = f"batt:v={msg.voltage:.1f},pct={msg.percentage*100:.0f}"
        self.get_logger().debug("Updated battery data")

    def dvl_velocity_callback(self, msg):
        lin = msg.twist.twist.linear
        self.latest_dvl_velocity = f"dvl_v:x={lin.x:.2f},y={lin.y:.2f},z={lin.z:.2f}"
        self.get_logger().debug("Updated DVL velocity data")

    def dvl_position_callback(self, msg):
        pos = msg.position
        self.latest_dvl_position = f"dvl_p:x={pos.x:.2f},y={pos.y:.2f},z={pos.z:.2f},r={msg.roll:.2f},p={msg.pitch:.2f},y={msg.yaw:.2f}"
        self.get_logger().debug("Updated DVL position data")

    def tx_callback(self, msg):
        try:
            message = msg.data
            self.device.send_data_broadcast(message)
            self.get_logger().debug(f"Sent via XBee: {message}")
        except Exception as e:
            self.get_logger().error(f"XBee transmission error: {str(e)}")
            self.get_logger().error(traceback.format_exc())

    def get_all_status_data(self):
        data_dict = {
            "odom": self.latest_odom,
            "leak": self.latest_leak,
            "battery": self.latest_battery,
            "dvl_vel": self.latest_dvl_velocity,
            "dvl_pos": self.latest_dvl_position,
        }
        data_dict = {k: v for k, v in data_dict.items() if v and v != "NO_DATA"}
        return json.dumps(data_dict, separators=(',', ':'))

    def data_receive_callback(self, xbee_message):
        try:
            payload = xbee_message.data.decode('utf-8', errors='replace')
            self.get_logger().info(f"Received from {xbee_message.remote_device.get_64bit_addr()}: {payload}")
            msg = String()
            msg.data = payload
            self.publisher.publish(msg)

            # Command handling (STATUS/INIT)
            if payload == "STATUS":
                response = self.get_all_status_data()
                self.device.send_data_broadcast(response)
                self.get_logger().info(f"Received STATUS, responding with sensor data")
                self.get_logger().debug(f"Status response: {response}")
            #TODO: when Braden adds the init system, update this
            elif payload == "INIT":
                init_msg = String()
                init_msg.data = "INIT_COMMAND"
                self.init_publisher.publish(init_msg)
                self.get_logger().info(f"Received INIT, published to init topic")
                self.device.send_data_broadcast("INIT_ACK")
        except Exception as e:
            self.get_logger().error(f"Error in data_receive_callback: {e}")

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
