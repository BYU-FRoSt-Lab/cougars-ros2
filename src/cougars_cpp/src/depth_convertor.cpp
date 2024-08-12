#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/PoseWithCovarianceStamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class DepthConvertor : public rclcpp::Node {
public:
  DepthConvertor() : Node("depth_convertor") {

    // declare ros publishers
    depth_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "calculated_depth", 10);

    // declare ros subscribers
    pressure_subscription_ =
        this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "depth_data", 10,
            std::bind(&DepthConvertor::pressure_callback, this, _1));
  }

private:
  void pressure_callback(
      const sensor_msgs::msg::FluidPressure::SharedPtr pressure_msg) {

    geometry_msgs::msg::PoseWithCovarianceStamped depth_msg;
    depth_msg.pose.position.z = pressure_msg->fluid_pressure; // TODO: convert to depth
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