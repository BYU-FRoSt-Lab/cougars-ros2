
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_msgs/msg/header.hpp"
#include <cmath>

using std::string;

/**
 * TODO: finish documenting
 * 
 * @brief A ros node that publishes static transforms
 * 
 * StaticTransformPublisher
 * 
 * defines the static transformations between 
 * 
 * Main Coordinate Frames:
 *  - robot: the coordinate frame at the center of the robot 
 *           x: forward, y: port, z: to top of vehicle
 * 
 *  - ENU: The coordinate frame of 
 * 
 * Sensor Coordinate Frames:
 *  - modem: the coordinate frame of the acoustic modem and its onboard imu
 *  - dvl: the coordinate frame of the dvl
 *  - 
 */
class StaticTransformPublisher : public rclcpp::Node {
public:

  StaticTransformPublisher() : Node("static_tf_publisher") {
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // The static transformation between robot and modem.
    // disregarding translation, the modem oriented in the same direction as the robot

    declare_static_tf_parameters("modem", "robot", {0.0,0.0,0.0}, {0.0,0.0,0.0,1.0});
    set_static_transform_from_param("modem", "robot", true);

    set_static_transform(
        "enu", "ned", false,
        0, 0, 0,
        std::sqrt(2)/2, std::sqrt(2)/2, 0, 0
    );

  }


  /**
   * 
   * 
   * @param from_frame
   * 
   * transformation is child frame wrt header frame
   * 
   */
  void set_static_transform(
    string const& header_frame, string const& child_frame, bool with_ns,
    float x,float y,float z,
    float qx, float qy, float qz, float qw
  ) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();

    std::string ns = with_ns? string(this->get_namespace())+".":""; 
    transform.header.frame_id = ns+header_frame;  // Parent frame
    transform.child_frame_id = ns+child_frame;   // Child frame

    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;

    transform.transform.rotation.x = qx;
    transform.transform.rotation.y = qy;
    transform.transform.rotation.z = qz;
    transform.transform.rotation.w = qw;

    broadcaster_->sendTransform(transform);
  }

  void declare_static_tf_parameters(
    string const& header_frame, std::string const& child_frame
  ) {
    this->declare_parameter<std::vector<double>>(header_frame+"."+child_frame+".translation");
    this->declare_parameter<std::vector<double>>(header_frame+"."+child_frame+".orientation");
  }
  void declare_static_tf_parameters(
    string const& header_frame, std::string const& child_frame,
    std::vector<double> const& default_translation, std::vector<double> const& default_orientation
  ) {
    this->declare_parameter<std::vector<double>>(header_frame+"."+child_frame+".translation", default_translation);
    this->declare_parameter<std::vector<double>>(header_frame+"."+child_frame+".orientation", default_orientation);
  }


  void set_static_transform_from_param(
    string const& header_frame, std::string const& child_frame, bool with_ns
  ) {
    std::vector<double> translation;
    this->get_parameter(header_frame+"."+child_frame+".translation", translation);
    std::vector<double> orientation;
    this->get_parameter(header_frame+"."+child_frame+".orientation", orientation);
    set_static_transform(
      header_frame, child_frame, with_ns,
      translation[0], translation[1], translation[2],
      orientation[0], orientation[1], orientation[2], orientation[3]
    );
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTransformPublisher>());
  rclcpp::shutdown();
  return 0;
}