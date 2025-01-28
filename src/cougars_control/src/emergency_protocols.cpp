#include <chrono>
#include <functional>
#include <memory>
#include <string>

// #include "pid.cpp"
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile);

/**
 * @brief A node for handling control for errors in vehicle system
 * @author Matthew McMurray
 * @date Jan 2025
 *
 */
class EmergencyProtocols : public rclcpp::Node {
public:

  /**
   * Creates a new controls node.
   */
  EmergencyProtocols() : Node("emergency_protocols") {

  
  }

private:
  

  
  



};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyProtocols>());
  rclcpp::shutdown();
  return 0;
}
