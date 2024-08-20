#include "rclcpp/rclcpp.hpp"
#include "dvl_msgs/msg/command_response.hpp"

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

        // Create a subscriber to the dvl/command/response topic with the QoS settings
        dvl_sub_command_response_ = this->create_subscription<dvl_msgs::msg::CommandResponse>(
            "dvl/command/response",
            qos,
            std::bind(&DvlCommandResponseListener::response_callback, this, std::placeholders::_1)
        );
    }

private:
    // Callback function to handle incoming messages
    void response_callback(const dvl_msgs::msg::CommandResponse::SharedPtr msg)
    {
        // Log the received message contents
        RCLCPP_INFO(this->get_logger(), "Received CommandResponse: success = %s, message = %s",
                    msg->success ? "true" : "false",
                    msg->message.c_str());
    }

    // Subscriber object
    rclcpp::Subscription<dvl_msgs::msg::CommandResponse>::SharedPtr dvl_sub_command_response_;
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
