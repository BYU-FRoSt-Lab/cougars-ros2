
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
    set_static_transform(
        "modem", "robot_orientation",
        0, 0, 0,
        0, 0, 0, 1
    );

    set_static_transform(
        "enu", "ned",
        0, 0, 0,
        std::sqrt(2)/2, std::sqrt(2)/2, 0, 0
    );

  }


  /**
   * 
   * 
   * @param from_frame
   * 
   */
  void set_static_transform(
    string const& from_frame, string const& to_frame,
    float x,float y,float z,
    float qx, float qy, float qz, float qw
  ) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();

    transform.header.frame_id = from_frame;  // Parent frame
    transform.child_frame_id = to_frame;   // Child frame

    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;

    transform.transform.rotation.x = qx;
    transform.transform.rotation.y = qy;
    transform.transform.rotation.z = qz;
    transform.transform.rotation.w = qw;

    broadcaster_->sendTransform(transform);
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