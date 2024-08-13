#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include "frost_interfaces/msg/u_command.hpp"
#include <libserialport.h>
#include <string>
#include <sstream>

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        // Initialize subscriber
        // TODO: Look at remappings for holoocean namespace??
        control_command_sub_ = this->create_subscription<frost_interfaces::msg::UCommand>(
            "control_command", 10, std::bind(&ControlNode::controlCommandCallback, this, std::placeholders::_1));

        // Initialize publisher
        pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("pressure_data", 10);

        // TODO: Make a new publisher that converts fluid pressure to depth with a pose with cov

        // Open the serial port
        if (sp_get_port_by_name("/dev/frost/control", &serial_port_) != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Unable to find port");
            rclcpp::shutdown();
        }

        if (sp_open(serial_port_, SP_MODE_READ_WRITE) != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port");
            sp_free_port(serial_port_);
            rclcpp::shutdown();
        }

        sp_set_baudrate(serial_port_, 115200);

        RCLCPP_INFO(this->get_logger(), "Serial port initialized");

        // Timer to read from serial port
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::readSerialData, this));
    }

    ~ControlNode() {
        sp_close(serial_port_);
        sp_free_port(serial_port_);
    }

private:
    void controlCommandCallback(const your_package::msg::UCommand::SharedPtr msg) {
        std::stringstream ss;
        // (Currently have 3 fins)
        ss << "$CONTR," << msg->fin[0] << "," << msg->fin[1] << "," << msg->fin[2] << "," << msg->thruster << "\n";
        std::string command = ss.str();
        sp_nonblocking_write(serial_port_, command.c_str(), command.size());
        RCLCPP_INFO(this->get_logger(), "Sent command: %s", command.c_str());
    }

    void readSerialData() {
        char buffer[100];
        int bytes_read = sp_nonblocking_read(serial_port_, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0'; // Null terminate the buffer
            std::string line(buffer);
            RCLCPP_INFO(this->get_logger(), "Received: %s", line.c_str());

            if (line.rfind("$DEPTH", 0) == 0) {
                double pressure, temperature;
                if (sscanf(line.c_str(), "$DEPTH,%lf,%lf", &pressure, &temperature) == 2) {
                    sensor_msgs::msg::FluidPressure pressure_msg;
                    pressure_msg.header.stamp = this->now();
                    pressure_msg.fluid_pressure = pressure;
                    
                    pressure_msg.variance = 0.0;  // Variance can be set if known
                    
                    pressure_pub_->publish(pressure_msg);
                    RCLCPP_INFO(this->get_logger(), "Published pressure: %lf, temperature: %lf", pressure, temperature);
                }
            }
        }
    }

    rclcpp::Subscription<frost_interfaces::msg::UCommand>::SharedPtr control_command_sub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct sp_port *serial_port_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
