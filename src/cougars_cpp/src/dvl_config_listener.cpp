#include "rclcpp/rclcpp.hpp"
#include "dvl_msgs/msg/command_response.hpp"
#include "dvl_msgs/msg/config_status.hpp"  // Assuming ConfigStatus.msg is generated into this header

class DvlCommandResponseListener : public rclcpp::Node
{
public:
    DvlCommandResponseListener()
        : Node("dvl_command_response_listener")
    {
        // Use rmw_qos_profile_sensor_data for QoS settings
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
                qos_profile.history,
                qos_profile.depth),
            qos_profile);

        // Subscriber for dvl/command/response topic
        dvl_sub_command_response_ = this->create_subscription<dvl_msgs::msg::CommandResponse>(
            "dvl/command/response",
            qos,
            std::bind(&DvlCommandResponseListener::response_callback, this, std::placeholders::_1)
        );

        // Subscriber for dvl/config/status topic
        dvl_sub_config_status_ = this->create_subscription<dvl_msgs::msg::ConfigStatus>(
            "dvl/config/status",
            qos,
            std::bind(&DvlCommandResponseListener::config_status_callback, this, std::placeholders::_1)
        );
    }

private:
    // Callback function to handle incoming messages from dvl/command/response
    void response_callback(const dvl_msgs::msg::CommandResponse::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received CommandResponse:");
        RCLCPP_INFO(this->get_logger(), "  response_to: %s", msg->response_to.c_str());
        RCLCPP_INFO(this->get_logger(), "  success: %s", msg->success ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  error_message: %s", msg->error_message.c_str());
        RCLCPP_INFO(this->get_logger(), "  result: %d", msg->result);
        RCLCPP_INFO(this->get_logger(), "  format: %s", msg->format.c_str());
        RCLCPP_INFO(this->get_logger(), "  type: %s", msg->type.c_str());
    }

    // Callback function to handle incoming messages from dvl/config/status
    void config_status_callback(const dvl_msgs::msg::ConfigStatus::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received ConfigStatus:");
        RCLCPP_INFO(this->get_logger(), "  response_to: %s", msg->response_to.c_str());
        RCLCPP_INFO(this->get_logger(), "  success: %s", msg->success ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  error_message: %s", msg->error_message.c_str());
        RCLCPP_INFO(this->get_logger(), "  speed_of_sound: %d", msg->speed_of_sound);
        RCLCPP_INFO(this->get_logger(), "  acoustic_enabled: %s", msg->acoustic_enabled ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  dark_mode_enabled: %s", msg->dark_mode_enabled ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  mounting_rotation_offset: %d", msg->mounting_rotation_offset);
        RCLCPP_INFO(this->get_logger(), "  range_mode: %s", msg->range_mode.c_str());
        RCLCPP_INFO(this->get_logger(), "  format: %s", msg->format.c_str());
        RCLCPP_INFO(this->get_logger(), "  type: %s", msg->type.c_str());
    }

    // Subscriber objects
    rclcpp::Subscription<dvl_msgs::msg::CommandResponse>::SharedPtr dvl_sub_command_response_;
    rclcpp::Subscription<dvl_msgs::msg::ConfigStatus>::SharedPtr dvl_sub_config_status_;
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 system
    rclcpp::init(argc, argv);

    // Create the node and spin it
    rclcpp::spin(std::make_shared<DvlCommandResponseListener>());

    // Shutdown the ROS 2 system
    rclcpp::shutdown();

    return 0;
}
