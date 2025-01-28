#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "frost_interfaces/msg/leak_status.hpp"
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
   * Creates a new emergency protocols node.
   */
  EmergencyProtocols() : Node("emergency_protocols") {



    // Emergencies below


    // leak
    leak_status_subscription_ =
        this->create_subscription<frost_interfaces::msg::LeakStatus>(
            "leak/data", 10,
            std::bind(&EmergencyProtocols::leak_callback, this, _1));

  }

private:



//

 void
  leak_callback(const frost_interfaces::msg::LeakStatus &leak_msg) {


    
  }






// add subscribers here





  rclcpp::Subscription<frost_interfaces::msg::LeakStatus>::SharedPtr
      leak_status_subscription_;
  
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyProtocols>());
  rclcpp::shutdown();
  return 0;
}
