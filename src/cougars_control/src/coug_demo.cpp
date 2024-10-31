#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "frost_interfaces/msg/u_command.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile);

/**
 * @brief CougUV demo node.
 * @author Nelson Durrant
 * @date October 2024
 *
 * This node subscribes to the "controls/command" topic and modifies the
 * commands from HoloOcean to run the demo safely. The modified commands
 * are then published to the "kinematics/command" topic.
 *
 * Subscribes:
 * - controls/command (frost_interfaces/msg/UCommand)
 * Publishes:
 * - kinematics/command (frost_interfaces/msg/UCommand)
 */
class CougDemo : public rclcpp::Node {
public:
  /**
   * @brief Creates a new demo node.
   *
   * This constructor creates a new demo node with default values.
   */
  CougDemo() : Node("coug_demo") {

    /**
     * @brief Demo command publisher.
     *
     * This publisher publishes the commands to the "kinematics/command" topic.
     * It uses the UCommand message type.
     */
    command_publisher_ =
        this->create_publisher<frost_interfaces::msg::UCommand>(
            "kinematics/command", 10);

    /**
     * @brief Controls command subscriber.
     *
     * This subscriber subscribes to the "controls/command" topic. It uses the
     * UCommand message type.
     */
    command_subscription_ =
        this->create_subscription<frost_interfaces::msg::UCommand>(
            "controls/command", 10,
            std::bind(&CougDemo::command_callback, this, _1));
  }

private:
  /**
   * @brief Callback function for the controls/command subscription.
   *
   * This method is called whenever a new controls command message is received.
   *
   * @param
   */
  void command_callback(const frost_interfaces::msg::UCommand &msg) {

    auto command = frost_interfaces::msg::UCommand();
    command.header.stamp = msg.header.stamp;
    command.fin[0] = msg.fin[0];
    command.fin[1] = -1 * msg.fin[1];
    command.fin[2] = msg.fin[2];
    command.thruster = 0;

    command_publisher_->publish(command);
  }

  // micro-ROS objects
  rclcpp::Publisher<frost_interfaces::msg::UCommand>::SharedPtr
      command_publisher_;
  rclcpp::Subscription<frost_interfaces::msg::UCommand>::SharedPtr
      command_subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CougKinematics>());
  rclcpp::shutdown();
  return 0;
}
