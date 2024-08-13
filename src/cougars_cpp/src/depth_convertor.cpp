#include <chrono>
#include <functional>
#include <memory>
#include <string>

#define GRAVITY 9.81
#define FLUID_DENSITY 1025
#define FLUID_PRESSURE_ATM 101325

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile);

class DepthConvertor : public rclcpp::Node {
public:
  DepthConvertor() : Node("depth_convertor") {

    // declare ros publishers
    depth_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "depth_data", 10);

    // declare ros subscribers
    pressure_subscription_ =
        this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "pressure_data", qos,
            std::bind(&DepthConvertor::pressure_callback, this, _1));
  }

private:
  void pressure_callback(
      const sensor_msgs::msg::FluidPressure::SharedPtr pressure_msg) {

    geometry_msgs::msg::PoseWithCovarianceStamped depth_msg;
    depth_msg.pose.pose.position.z =
        (pressure_msg->fluid_pressure - FLUID_PRESSURE_ATM) /
        (FLUID_DENSITY * GRAVITY);
    depth_publisher_->publish(depth_msg);
  }

  // micro-ROS objects
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      depth_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr
      pressure_subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthConvertor>());
  rclcpp::shutdown();
  return 0;
}