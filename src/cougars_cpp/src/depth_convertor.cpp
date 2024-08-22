#include <chrono>
#include <functional>
#include <memory>
#include <string>

#define GRAVITY 9.81
#define FLUID_DENSITY_BASE 997

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

    this->declare_parameter("water_salinity_ppt", 0.0); // 35.0 for salt water, 0.0 for fresh water
    this->declare_parameter("fluid_pressure_atm", 87250.0); // 87250.0 from testing

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
    // set time from pressure message
    depth_msg.header.stamp = pressure_msg->header.stamp;
    depth_msg.pose.pose.position.z =
        - (pressure_msg->fluid_pressure * 100 - this->get_parameter("fluid_pressure_atm").as_double()) /
        ((FLUID_DENSITY_BASE + this->get_parameter("water_salinity_ppt").as_double()) * GRAVITY);
    // RCLCPP_INFO(this->get_logger(), "Pressure: %f",
    //             pressure_msg->fluid_pressure);
    // RCLCPP_INFO(this->get_logger(), "Depth: %f",
    //             depth_msg.pose.pose.position.z);
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