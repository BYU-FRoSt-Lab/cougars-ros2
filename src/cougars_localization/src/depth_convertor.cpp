#include <chrono>
#include <functional>
#include <memory>
#include <string>

#define GRAVITY 9.81           // m/s^2
#define FLUID_DENSITY_BASE 997 // kg/m^3

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile);

/**
 * @brief A simple depth convertor node.
 * @author Nelson Durrant
 * @date September 2024
 *
 * This node subscribes to a pressure sensor topic and converts the pressure
 * data to depth data. The depth data is then published to a depth data topic.
 *
 * Subscribes:
 * - pressure/data (sensor_msgs/msg/FluidPressure)
 * Publishes:
 * - depth_data (geometry_msgs/msg/PoseWithCovarianceStamped)
 */
class DepthConvertor : public rclcpp::Node {
public:

  /**
   * Creates a new depth convertor node.
   */
  DepthConvertor() : Node("depth_convertor") {

    /**
     * @param water_salinity_ppt
     *
     * The salinity of the water in parts per thousand (ppt). The default value
     * is 0.0 ppt for fresh water. The value should be set to 35.0 ppt for salt
     * water.
     */
    this->declare_parameter("water_salinity_ppt", 0.0);

    /**
     * @param fluid_pressure_atm
     *
     * The atmospheric pressure in Pascals (Pa). The default value is 87250.0 Pa
     * from our lab's field testing.
     */
    this->declare_parameter("fluid_pressure_atm", 87250.0);

    /**
     * @brief Depth publisher.
     * 
     * This publisher publishes the depth data to the "depth_data" topic. It
     * uses the PoseWithCovarianceStamped message type.
     */
    depth_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "depth_data", 10);

    /**
     * @brief Pressure subscriber.
     * 
     * This subscriber subscribes to the "pressure/data" topic. It uses the
     * FluidPressure message type.
     */
    pressure_subscription_ =
        this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "pressure/data", qos,
            std::bind(&DepthConvertor::pressure_callback, this, _1));
  }

private:

  /**
   * @brief Callback function for the pressure subscription.
   * 
   * This method converts the pressure data to depth data using the formula:
   *
   * depth = (fluid_pressure_atm - pressure * 100) / (fluid_density +
   * water_salinity_ppt) * gravity)
   *
   * @param pressure_msg The pressure data message.
   */
  void pressure_callback(
      const sensor_msgs::msg::FluidPressure::SharedPtr pressure_msg) {

    geometry_msgs::msg::PoseWithCovarianceStamped depth_msg;
    depth_msg.header.stamp =
        pressure_msg->header.stamp; // copy exact time from the sensor message
    depth_msg.pose.pose.position.z =
        (this->get_parameter("fluid_pressure_atm").as_double() -
         pressure_msg->fluid_pressure * 100) /
        ((FLUID_DENSITY_BASE +
          this->get_parameter("water_salinity_ppt").as_double()) *
         GRAVITY);
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