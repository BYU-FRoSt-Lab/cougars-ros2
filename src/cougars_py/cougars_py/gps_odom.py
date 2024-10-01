import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix
from nav_msgs.msg import Odometry
from pyproj import Proj, Transformer
import math

class NavSatFixToOdom(Node):
    def __init__(self):
        super().__init__('gps_odom')
        
        # Declare parameters for the origin (datum)
        self.declare_parameter('origin.latitude', 34.0219)
        self.declare_parameter('origin.longitude', -118.4814)
        self.declare_parameter('origin.altitude', 0.0)
        
        # Get parameters
        origin_lat = self.get_parameter('origin.latitude').get_parameter_value().double_value
        origin_lon = self.get_parameter('origin.longitude').get_parameter_value().double_value
        self.origin_alt = self.get_parameter('origin.altitude').get_parameter_value().double_value
        
        # Create local Cartesian projection with the origin
        self.local_proj = Proj(proj='aeqd', lat_0=origin_lat, lon_0=origin_lon, datum='WGS84')
        
        # Create geographic projection
        self.geo_proj = Proj(proj='latlong', datum='WGS84')
        
        # Create transformers for conversion
        self.transformer = Transformer.from_proj(
            self.local_proj,
            self.geo_proj,
            always_xy=True
        )
        
        self.inv_transformer = Transformer.from_proj(
            self.geo_proj,
            self.local_proj,
            always_xy=True
        )
        
        # Subscribe to NavSatFix
        self.subscriber = self.create_subscription(
            GPSFix,
            '/extended_fix',
            self.gps_callback,
            10
        )
        
        self.last_msg = None
        self.min_sats = 5  # Minimum number of satellites

        # Publisher for Odometry
        self.publisher = self.create_publisher(Odometry, '/gps_odom', 10)
    
    def gps_callback(self, msg: GPSFix):
        # Filter out bad readings based on the number of satellites (if available)
        if msg.status.satellites_used < self.min_sats or msg.latitude == 0:
            self.get_logger().warn(f"Bad GPS status, skipping this GPS reading. Sat Used: {msg.status.satellites_used}")
            return
        
        if math.isnan(msg.latitude) or math.isnan(msg.longitude) or math.isnan(msg.altitude):
            self.get_logger().warn("NaN detected in GPS position, skipping this reading")
            return
        
        # Convert latitude/longitude to local Cartesian coordinates
        x, y = self.inv_transformer.transform(msg.longitude, msg.latitude)
        
        # Access the altitude (z) value from the NavSatFix message
        z = msg.altitude - self.origin_alt

        # Fill in the odometry message
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "odom"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z  # Use the altitude as the z-value

        # Set the covariance values for x, y, and z
        odom.pose.covariance[0] = msg.position_covariance[0]  # xx
        odom.pose.covariance[7] = msg.position_covariance[4]  # yy
        odom.pose.covariance[14] = msg.position_covariance[8]  # zz

        # Publish the odometry message
        self.last_msg = odom
        self.publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = NavSatFixToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
