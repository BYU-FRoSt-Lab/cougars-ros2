#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <seatrac_interfaces/msg/modem_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


class SeatracAHRSConverter : public rclcpp::Node {
public:
  SeatracAHRSConverter() : Node("seatrac_ahrs_converter") {
    // https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?
    this->declare_parameter("magnetic_declination",
                            10.7); // 10.70° E for Utah Lake
    magnetic_declination = this->get_parameter("magnetic_declination").as_double();

    modem_subscriber_ =
        this->create_subscription<seatrac_interfaces::msg::ModemStatus>(
            "modem_status", 10,
            std::bind(&SeatracAHRSConverter::modem_callback, this,
                      std::placeholders::_1));

    modem_imu_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>("modem_imu", 10);

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  }

private:
  void modem_callback(const seatrac_interfaces::msg::ModemStatus::SharedPtr msg) {
    auto modem_imu = std::make_shared<sensor_msgs::msg::Imu>();
    modem_imu->header.stamp = msg->header.stamp;

    double yaw   = (M_PI / 180.0) * (0.1 * msg->attitude_yaw + magnetic_declination);
    double pitch = (M_PI / 180.0) * 0.1 * msg->attitude_pitch;
    double roll  = (M_PI / 180.0) * 0.1 * msg->attitude_roll;

    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "ned";
    t.child_frame_id = "modem";

    // coordinates from the message and set the z coordinate to 0
    // t.transform.translation.x = 0.0;
    // t.transform.translation.y = 0.0;
    // t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    // Create the NED quaternion using Eigen
    Eigen::Matrix3d R_ned(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    // Define the NED to ENU conversion as a quaternion
    Eigen::Matrix3d R_ned_enu;
    R_ned_enu << 
        0, -1,  0, 
       -1,  0,  0, 
        0,  0, -1;

    // Convert to ENU coordinates by applying the NED-to-ENU rotation
    Eigen::Quaterniond q_enu(R_ned * R_ned_enu);

    modem_imu->orientation.x = q_enu.x();
    modem_imu->orientation.y = q_enu.y();
    modem_imu->orientation.z = q_enu.z();
    modem_imu->orientation.w = q_enu.w();


    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "enu";
    t.child_frame_id = "modem_check";
    t.transform.rotation.x = q_enu.x();
    t.transform.rotation.y = q_enu.y();
    t.transform.rotation.z = q_enu.z();
    t.transform.rotation.w = q_enu.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    geometry_msgs::msg::TransformStamped t_listen;

    try {
      t_listen = tf_buffer_->lookupTransform(
        "enu", "modem",
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "enu";
    t.child_frame_id = "robot";
    t.transform.rotation.x = t_listen.transform.rotation.x;
    t.transform.rotation.y = t_listen.transform.rotation.y;
    t.transform.rotation.z = t_listen.transform.rotation.z;
    t.transform.rotation.w = t_listen.transform.rotation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    // modem_imu->orientation_covariance = {
    //     1.0, 0.0, 0.0,
    //     0.0, 1.0, 0.0, 
    //     0.0, 0.0, 1.0
    // };


    // no coordinate conversions needed because modem coordinate frame
    // is the same as the regular coordinate frame
    modem_imu->angular_velocity.x = msg->gyro_x;
    modem_imu->angular_velocity.y = msg->gyro_y;
    modem_imu->angular_velocity.z = msg->gyro_z;


    modem_imu->linear_acceleration.x = msg->acc_x;
    modem_imu->linear_acceleration.y = msg->acc_y;
    modem_imu->linear_acceleration.z = msg->acc_z;

    // Publish the IMU message
    modem_imu_pub_->publish(*modem_imu);

  }

  rclcpp::Subscription<seatrac_interfaces::msg::ModemStatus>::SharedPtr
      modem_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr modem_imu_pub_;
  double magnetic_declination;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SeatracAHRSConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}