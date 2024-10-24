
.. _program_listing_file_src_vehicle_status.cpp:

Program Listing for File vehicle_status.cpp
===========================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_vehicle_status.cpp>` (``src/vehicle_status.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   
   
   #include <chrono>
   #include <functional>
   #include <memory>
   #include <string>
   
   #include "dvl_msgs/msg/dvl.hpp"
   #include "dvl_msgs/msg/dvldr.hpp"
   #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
   #include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/float64.hpp"
   #include "std_msgs/msg/string.hpp"
   
   #include "nav_msgs/msg/odometry.hpp"
   #include <vector>
   
   #include <chrono>
   #include <functional>
   #include <memory>
   #include <string>
   
   #include "dvl_msgs/msg/dvldr.hpp"
   #include "frost_interfaces/msg/desired_depth.hpp"
   #include "frost_interfaces/msg/desired_heading.hpp"
   #include "frost_interfaces/msg/desired_speed.hpp"
   #include "seatrac_interfaces/msg/modem_status.hpp"
   
   #include "frost_interfaces/msg/u_command.hpp"
   #include "frost_interfaces/msg/vehicle_status.hpp"
   #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
   #include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
   #include "rclcpp/rclcpp.hpp"
   
   #define UPDATE_TIMER_MS std::chrono::milliseconds(10)
   
   #define PI_NUM 3.141592653589
   
   using namespace std::chrono_literals;
   using std::placeholders::_1;
   
   rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
   auto qos = rclcpp::QoS(
       rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
       qos_profile);
   
   class VehicleStatus : public rclcpp::Node {
   
   
   public:
     VehicleStatus() : Node("vehicle_status") {
   
       // 10.7 for Utah lake
       this->declare_parameter("magnetic_declination", 10.7);
       this->magnetic_declination = this->get_parameter("magnetic_declination").as_double();
   
       x_y_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
           "/smoothed_output", 10,
           std::bind(&VehicleStatus::x_y_callback, this, _1));
   
       depth_subscription_ = this->create_subscription<
           geometry_msgs::msg::PoseWithCovarianceStamped>(
           "depth_data", 10, std::bind(&VehicleStatus::depth_callback, this, _1));
   
       velocity_subscription_ = this->create_subscription<
           geometry_msgs::msg::TwistWithCovarianceStamped>(
           "dvl_velocity", 10,
           std::bind(&VehicleStatus::velocity_callback, this, _1));
   
       // orientation_subscription_ =
       // this->create_subscription<sensor_msgs::msg::Imu>(
       //     "modem_imu", qos, std::bind(&VehicleStatus::orientation_callback,
       //     this, _1));
   
       modem_yaw_subscription_ =
           this->create_subscription<seatrac_interfaces::msg::ModemStatus>(
               "modem_status", qos,
               std::bind(&VehicleStatus::modem_yaw_callback, this, _1));
   
       update_timer_ = this->create_wall_timer(
           UPDATE_TIMER_MS,
           std::bind(&VehicleStatus::broadcast_status_callback, this));
   
       vehicle_status_publisher_ =
           this->create_publisher<frost_interfaces::msg::VehicleStatus>(
               "vehicle_status", 10);
     }
   
   private:
     void x_y_callback(const nav_msgs::msg::Odometry &x_y_message) {
       this->y_pos = x_y_message.pose.pose.position.y;
       this->x_pos = x_y_message.pose.pose.position.x;
     }
     void depth_callback(
         const geometry_msgs::msg::PoseWithCovarianceStamped &depth_msg) {
       this->depth = depth_msg.pose.pose.position.z;
     }
     void velocity_callback(
         const geometry_msgs::msg::TwistWithCovarianceStamped &velocity_msg) {
       this->x_velocity = velocity_msg.twist.twist.linear.x;
     }
     // void orientation_callback(const
     // geometry_msgs::msg::PoseWithCovarianceStamped &orientation_msg) {
     //   this->q_w = orientation_msg.pose.pose.orientation.w;
     //   this->q_x = orientation_msg.pose.pose.orientation.x;
     //   this->q_y = orientation_msg.pose.pose.orientation.y;
     //   this->q_z = orientation_msg.pose.pose.orientation.z;
     // }
   
     void modem_yaw_callback(const seatrac_interfaces::msg::ModemStatus &yaw_msg) {
       //this yaw is in radians west of true north between -pi and pi
       //TODO: make sure this is what we want
       // (Note: MOOS defines yaw to be negative heading)
       this->yaw = -(PI_NUM / 180.0) * (0.1*yaw_msg.attitude_yaw + this->magnetic_declination);
     }
   
     void broadcast_status_callback() {
       auto message = frost_interfaces::msg::VehicleStatus();
       message.header.stamp = this->now();
       message.coug_odom.pose.pose.position.x = this->x_pos;
       message.coug_odom.pose.pose.position.y = this->y_pos;
       message.coug_odom.pose.pose.position.z = this->depth;
       // message.coug_odom.pose.pose.orientation.w = this->q_w;
       // message.coug_odom.pose.pose.orientation.y = this->q_y;
       // message.coug_odom.pose.pose.orientation.z = this->q_z;
       // message.coug_odom.pose.pose.orientation.x = this->q_x;
       message.coug_odom.twist.twist.linear.x = this->x_velocity;
       message.attitude_yaw = this->yaw;
   
       // publishes speed, depth, global x,y,
       // and orientation (quaternion)
       // to be used by MOOS and anything else to
       vehicle_status_publisher_->publish(message);
     }
   
     // call back timer to update
     rclcpp::TimerBase::SharedPtr update_timer_;
     // depth, speed, heading
     rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
         depth_subscription_;
     rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::
         SharedPtr velocity_subscription_;
     // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
     // orientation_subscription_;
   
     // current x,y
     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr x_y_subscription_;
   
     rclcpp::Subscription<seatrac_interfaces::msg::ModemStatus>::SharedPtr
         modem_yaw_subscription_;
   
     rclcpp::Publisher<frost_interfaces::msg::VehicleStatus>::SharedPtr
         vehicle_status_publisher_;
   
     //magnetic declination
     double magnetic_declination;
   
     // status variables
     float q_x = 0.0;
     float q_y = 0.0;
     float q_z = 0.0;
     float q_w = 0.0;
     float x_velocity = 0.0;
     float depth = 0.0;
     float x_pos = 0.0;
     float y_pos = 0.0;
     float yaw = 0.0;
   };
   
   int main(int argc, char *argv[]) {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<VehicleStatus>());
     rclcpp::shutdown();
     return 0;
   }
