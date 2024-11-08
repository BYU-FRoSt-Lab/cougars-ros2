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
 * @brief CougUV kinematics node.
 * @author Nelson Durrant
 * @date September 2024
 *
 * This node subscribes to the "controls/command" topic and modifies the
 * commands to account for thruster trim and fin offsets. The modified commands
 * are then published to the "kinematics/command" topic.
 *
 * Subscribes:
 * - controls/command (frost_interfaces/msg/UCommand)
 * Publishes:
 * - kinematics/command (frost_interfaces/msg/UCommand)
 */
class CougKinematics : public rclcpp::Node {
public:
  /**
   * @brief Creates a new kinematics node.
   *
   * This constructor creates a new kinematics node with default values.
   */
  CougKinematics() : Node("coug_kinematics") {

    /**
     * @param trim_ratio
     *
     * The trim ratio is used to adjust the control commands to account for
     * thruster trim. The default value is 0.0.
     */
    this->declare_parameter("trim_ratio", 0.0);

    /**
     * @param top_fin_offset
     *
     * The offset for the top fin to make it align properly. The default value
     * is 0.0.
     */
    this->declare_parameter("top_fin_offset", 0.0);

    /**
     * @param right_fin_offset
     *
     * The offset for the right fin (from the front) to make it align properly.
     * The default value is 0.0.
     */
    this->declare_parameter("right_fin_offset", 0.0);

    /**
     * @param left_fin_offset
     *
     * The offset for the left fin (from the front) to make it align properly.
     * The default value is 0.0.
     */
    this->declare_parameter("left_fin_offset", 0.0);

    /**
     * @param demo_mode
     * 
     * If true, the node will disable the thruster.
     * The default value is false.
     */
    this->declare_parameter("demo_mode", false);

    /**
     * @brief Kinematics command publisher.
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
            std::bind(&CougKinematics::command_callback, this, _1));
  }

private:
  /**
   * @brief Callback function for the controls/command subscription.
   *
   * This method is called whenever a new controls command message is received.
   * It adds the declared offsets and trim ratio to the commands and publishes
   * the new modified commands.
   *
   * @param
   */
  void command_callback(const frost_interfaces::msg::UCommand &msg) {

    auto command = frost_interfaces::msg::UCommand();
    command.header.stamp = msg.header.stamp;
    command.fin[0] =
        msg.fin[0] + this->get_parameter("top_fin_offset").as_double() +
        this->get_parameter("trim_ratio").as_double() * msg.thruster;
    command.fin[1] = -1 *
        msg.fin[1] + this->get_parameter("right_fin_offset").as_double() +
        this->get_parameter("trim_ratio").as_double() * msg.thruster;
    command.fin[2] =
        msg.fin[2] + this->get_parameter("left_fin_offset").as_double() +
        this->get_parameter("trim_ratio").as_double() * msg.thruster;

    if (this->get_parameter("demo_mode").as_bool()) {
      command.thruster = 0;
    } else {
      command.thruster = msg.thruster;
    }

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
