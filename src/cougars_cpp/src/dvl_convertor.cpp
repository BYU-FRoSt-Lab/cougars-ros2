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
#include <vector>

#include <math.h>
#include <stdint.h>
using std::placeholders::_1;

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile);

class DVLConvertor : public rclcpp::Node {
public:
  DVLConvertor() : Node("dvl_convertor") {
    publisher_dvl_depth =
        this->create_publisher<std_msgs::msg::Float64>("dvl_dfb", 10);
    publisher_dvl_velocity =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "dvl_velocity", 10);
    publisher_dvl_dead_reckoning =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "dvl_dead_reckoning", 10);
    subscriber_dvl_data = this->create_subscription<dvl_msgs::msg::DVL>(
        "/dvl/data", qos,
        std::bind(&DVLConvertor::dvl_data_callback, this, _1));
    subscriber_dvl_position = this->create_subscription<dvl_msgs::msg::DVLDR>(
        "/dvl/position", qos,
        std::bind(&DVLConvertor::dvl_pos_callback, this, _1));
  }

  void dvl_data_callback(const dvl_msgs::msg::DVL::SharedPtr msg) {

    geometry_msgs::msg::TwistWithCovarianceStamped stamped_msg;

    // filling in the upper left corner of the 6X6 covariance matrix
    int index = 0;
    double defaultValue = 0;
    for (int i = 0; i < 36; i++) {
      if (i % 6 < 3 && i < 15) {
        stamped_msg.twist.covariance[i] = msg->covariance[index];
        index++;
      } else {
        stamped_msg.twist.covariance[i] = defaultValue;
      }
    }

    stamped_msg.twist.twist.linear = msg->velocity;
    // negate z and y -- will this mess with covariance?
    stamped_msg.twist.twist.linear.y = -1.0 * msg->velocity.y;
    stamped_msg.twist.twist.linear.y = -1.0 * msg->velocity.z;
    publisher_dvl_velocity->publish(stamped_msg);
  }


  void dvl_pos_callback(const dvl_msgs::msg::DVLDR::SharedPtr msg) {

    geometry_msgs::msg::PoseWithCovarianceStamped stamped_msg;

    stamped_msg.header.frame_id = 'odom'; //odom is the coordinate frame of the dvl in robot_localization

    stamped_msg.pose.pose.position.x = msg->position.x;
    stamped_msg.pose.pose.position.y = -1.0 * msg->position.y;
    stamped_msg.pose.pose.position.z = -1.0 * msg->position.z;

    //TODO: make full quaternion (not just yaw)
    stamped_msg.pose.pose.orientation.x = 0.0;
    stamped_msg.pose.pose.orientation.y = 0.0;
    stamped_msg.pose.pose.orientation.z = sin(0.5*msg->yaw*M_PI/180.0); // q = cos(theta/2) + sin(theta/2)(xi+yj+zk)
    stamped_msg.pose.pose.orientation.w = cos(0.5*msg->yaw*M_PI/180.0);

    //TODO: Check and tune covariance parameters
    double pvr = msg->pos_std * msg->pos_std; //variance = std squared
    double yvr = 1.0; //An estimate for the variance in yaw
    stamped_msg.pose.covariance = { 
      pvr, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, pvr, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, pvr, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, yvr,
    };

    publisher_dvl_dead_reckoning->publish(stamped_msg);
  }

private:
  // publisher localizatoin pkg types
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      publisher_dvl_velocity;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      publisher_dvl_dead_reckoning;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_dvl_depth;

  // subscribers - listening to dvl driver
  rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr subscriber_dvl_data;
  rclcpp::Subscription<dvl_msgs::msg::DVLDR>::SharedPtr subscriber_dvl_position;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DVLConvertor>());
  rclcpp::shutdown();
  return 0;
}