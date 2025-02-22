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


#define SQRT_OF_2_OVER_2 0.70710678118
#define PI 3.14159265359 
#define DEGREES_TO_RADIANS PI/180.0

//From experimentation, 1 G, equivelant to 9.81 m/s^2, is about 250 in raw accelerometer units
#define ACC_UNITS_TO_METERS_PER_SECOND_SQUARED 9.80665 / 250.0


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

    double yaw   = (M_PI / 180.0) * (0.1 * msg->attitude_yaw + magnetic_declination);
    double pitch = (M_PI / 180.0) * 0.1 * msg->attitude_pitch;
    double roll  = (M_PI / 180.0) * 0.1 * msg->attitude_roll;

    //==========================//
    // tf transform message     //
    //==========================//


    geometry_msgs::msg::TransformStamped t;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "ned";
    t.child_frame_id = "modem";

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);


    //==========================//
    //   get tf enu to robot    //
    //==========================//


    geometry_msgs::msg::TransformStamped t_listen;

    // try {
    //   t_listen = tf_buffer_->lookupTransform(
    //     "enu", "modem",
    //     tf2::TimePointZero);
    // } catch (const tf2::TransformException & ex) {
    //   RCLCPP_INFO(
    //     this->get_logger(), "Could not transform: %s", ex.what());
    //   return;
    // }


    //==========================//
    //   publish as modem_imu   //
    //==========================//

    frost_interfaces::msg::ModemImu::SharedPtr modem_imu;

    modem_imu->



    // The gyroscope is mounted with a 45 degree offset from the modem frame
    // So a linear transformation is needed. Determined experimentally.
    modem_imu->angular_velocity.x = DEGREES_TO_RADIANS*SQRT_OF_2_OVER_2*(msg->gyro_y - msg->gyro_x);
    modem_imu->angular_velocity.y = DEGREES_TO_RADIANS*SQRT_OF_2_OVER_2*(msg->gyro_x + msg->gyro_y);
    modem_imu->angular_velocity.z = -DEGREES_TO_RADIANS*msg->gyro_z;

    // The accelerometer is mounted with a 45 degree offset from the modem frame
    // so a linear transformation is need. Determined experimentally.
    modem_imu->linear_acceleration.x =  ACC_UNITS_TO_METERS_PER_SECOND_SQUARED * SQRT_OF_2_OVER_2*(msg->acc_x - msg->acc_y);
    modem_imu->linear_acceleration.y = -ACC_UNITS_TO_METERS_PER_SECOND_SQUARED * SQRT_OF_2_OVER_2*(msg->acc_x + msg->acc_y);
    modem_imu->linear_acceleration.z =  ACC_UNITS_TO_METERS_PER_SECOND_SQUARED * msg->acc_z;

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