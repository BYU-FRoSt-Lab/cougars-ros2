import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from pyproj import Proj, Transformer

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
        origin_alt = self.get_parameter('origin.altitude').get_parameter_value().double_value
        
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
        
        # Subscribe to PoseWithCovarianceStamped
        self.subscriber = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10
        )
        self.last_msg = None
        self.gps_last_time = None
        self.gps_covariance = [1e-3] * 9  # Initial low covariance
        self.timeout_duration = 3.0  

        # self.timer = self.create_timer(1.0, self.check_gps_timeout)
        # Publisher for Odometry
        self.publisher = self.create_publisher(Odometry, '/gps_odom', 10)
    
    def check_gps_timeout(self):
        if self.gps_last_time is not None:
            elapsed_time = (self.get_clock().now() - self.gps_last_time).nanoseconds / 1e9
            if elapsed_time > self.timeout_duration:
                # GPS timeout - increase covariance
                self.gps_covariance = [1e6] * 36
                # Publish/update the EKF covariance accordingly
                self.last_msg.pose.covariance = self.gps_covariance

                self.publisher.publish(self.last_msg)
            else:
                # GPS is available, keep low covariance
                self.gps_covariance = [1e-3] * 36
            
    
    def gps_callback(self, msg: NavSatFix):
        self.gps_last_time = self.get_clock().now()
        # Convert latitude/longitude to UTM or another flat-earth coordinate system
        x, y = self.inv_transformer.transform(msg.longitude ,msg.latitude )
        
        # Fill in the odometry message
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "odom"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        odom.pose.covariance[0] = msg.position_covariance[0] # xx
        # msg.position_covariance[1] = odom.pose.covariance[1]  # xy
        # msg.position_covariance[2] = odom.pose.covariance[5]  # xz
        # msg.position_covariance[3] = odom.pose.covariance[6]  # yx
        odom.pose.covariance[7] = msg.position_covariance[4]  # yy
        # msg.position_covariance[5] = odom.pose.covariance[11] # yz
        # msg.position_covariance[6] = odom.pose.covariance[30] # zx
        # msg.position_covariance[7] = odom.pose.covariance[31] # zy
        odom.pose.covariance[14]  = msg.position_covariance[8] # zz
        
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

