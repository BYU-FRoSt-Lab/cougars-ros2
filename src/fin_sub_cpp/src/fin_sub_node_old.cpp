#include <memory>
#include <iostream>
#include <sstream>
#include <libserialport.h>
#include "rclcpp/rclcpp.hpp"
#include "holoocean_interfaces/msg/u_command.hpp"

using std::placeholders::_1;

class FinSubscriber : public rclcpp::Node
{
public:
    FinSubscriber()
    : Node("fin_sub_node")
    {
        // Initialize the serial port
        std::string port_name = "/dev/ttyACM0";
        RCLCPP_INFO(this->get_logger(), "Attempting to open serial port: %s", port_name.c_str());

        check_error(sp_get_port_by_name(port_name.c_str(), &port_));
        check_error(sp_open(port_, SP_MODE_WRITE));
        check_error(sp_set_baudrate(port_, 115200));
        check_error(sp_set_bits(port_, 8));
        check_error(sp_set_parity(port_, SP_PARITY_NONE));
        check_error(sp_set_stopbits(port_, 1));
        check_error(sp_set_flowcontrol(port_, SP_FLOWCONTROL_NONE));

        RCLCPP_INFO(this->get_logger(), "Serial port successfully opened and configured");

        subscription_ = this->create_subscription<holoocean_interfaces::msg::UCommand>(
        "holoocean/ControlCommand", 10, std::bind(&FinSubscriber::topic_callback, this, _1));
    }

    ~FinSubscriber() {
        // Close the serial port
        if (port_) {
            sp_close(port_);
            sp_free_port(port_);
        }
    }

private:
    struct sp_port *port_;

    void check_error(enum sp_return result) const {
        if (result != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", sp_last_error_message());
            sp_free_error_message(sp_last_error_message());
            rclcpp::shutdown();
            exit(1);
        }
    }

    void topic_callback(const holoocean_interfaces::msg::UCommand & msg) const
    {
        // Extract fin values (Currently have 3 fins)
        float fin1 = msg.fin[0];
        float fin2 = msg.fin[1];
        float fin3 = msg.fin[2];

        // Create a string message to send to the microcontroller
        std::ostringstream oss;
        oss << "fin1:" << fin1 << ",fin2:" << fin2 << ",fin3:" << fin3 << "\n";
        std::string data = oss.str();

        // Write data to the serial port
        int bytes_written = sp_blocking_write(port_, data.c_str(), data.length(), 1000);
        if (bytes_written < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port");
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent %d bytes: %s", bytes_written, data.c_str());
        }
    }

    rclcpp::Subscription<holoocean_interfaces::msg::UCommand>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FinSubscriber>());
    rclcpp::shutdown();
    return 0;
}
