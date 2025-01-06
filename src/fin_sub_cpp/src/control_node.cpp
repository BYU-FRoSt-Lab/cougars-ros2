#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include "frost_interfaces/msg/u_command.hpp"
#include <libserialport.h>
#include <string>
#include <sstream>
// Added these libraries for the real path resolution
#include <stdlib.h>
#include <limits.h>

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node") {
        // Initialize subscriber
        // TODO: Look at remappings for holoocean namespace??
        control_command_sub_ = this->create_subscription<frost_interfaces::msg::UCommand>(
            "kinematics/command", 10, std::bind(&ControlNode::controlCommandCallback, this, std::placeholders::_1));

        // Initialize publisher
        pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("pressure/data", 10);

        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery/data", 10);

        leak_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("leak/data", 10);
        // TODO: Make a new publisher that converts fluid pressure to depth with a pose with cov

        sp_set_baudrate(serial_port_, 9600);
        // Open the serial port


        // Added this code so that we can use symlink path

        //This function finds the real path within the udev rule so that it can use the real path with the libserialport library
        //libserial port library cannot use udev paths
        char resolved_path[PATH_MAX];
        if (realpath("/dev/frost/teensy", resolved_path) != NULL) {
            std::cout << "Real path: " << resolved_path << std::endl;
        } else {
            std::cerr << "Error resolving path" << std::endl;
        }
        
        if (sp_get_port_by_name(resolved_path, &serial_port_) != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Unable to find port");
            rclcpp::shutdown();
        }

        if (sp_open(serial_port_, SP_MODE_READ_WRITE) != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port");
            sp_free_port(serial_port_);
            rclcpp::shutdown();
        }


        RCLCPP_INFO(this->get_logger(), "Serial port initialized");

        // Timer to read from serial port
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::readSerialData, this));
    }

    ~ControlNode() {
        sp_close(serial_port_);
        sp_free_port(serial_port_);
    }

private:
    void controlCommandCallback(const frost_interfaces::msg::UCommand::SharedPtr msg) {
        std::stringstream ss;
        // (Currently have 3 fins)
        ss << "$CONTR," << msg->fin[0] << "," << msg->fin[1] << "," << msg->fin[2] << "," << msg->thruster << "\n";
        std::string command = ss.str();
        sp_nonblocking_write(serial_port_, command.c_str(), command.size());
        // RCLCPP_INFO(this->get_logger(), "Sent command: %s", command.c_str());
        std::cout << "Sent command: " << command.c_str() << std::endl;
    }

    void readSerialData() {
        char temp_buffer[1024];
        int bytes_read = sp_nonblocking_read(serial_port_, temp_buffer, sizeof(temp_buffer) - 1);
        
        if (bytes_read > 0) {
            temp_buffer[bytes_read] = '\0';  // Null terminate the buffer
            buffer_ += temp_buffer;  // Append to existing buffer

            std::cout << "Received: " << temp_buffer << std::endl;

            std::vector<std::string> messages = splitMessages(buffer_);
            
            // Process complete messages
            for (size_t i = 0; i < messages.size() - 1; ++i) {
                processMessage(messages[i]);
            }
            
            // Keep the last (potentially incomplete) message in the buffer
            buffer_ = messages.empty() ? "" : messages.back();
        } else if (bytes_read < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error reading from serial port");
        }
    }

    std::vector<std::string> splitMessages(const std::string& data) {
        std::vector<std::string> messages;
        std::istringstream stream(data);
        std::string message;
        while (std::getline(stream, message, delimiter_)) {
            if (!message.empty()) {
                messages.push_back(message);
            }
        }
        return messages;
    }

    void processMessage(const std::string& message) {
        if (message.rfind("$DEPTH", 0) == 0) {
            double pressure, temperature;
            if (sscanf(message.c_str(), "$DEPTH,%lf,%lf", &pressure, &temperature) == 2) {
                sensor_msgs::msg::FluidPressure pressure_msg;
                pressure_msg.header.stamp = this->now();
                pressure_msg.fluid_pressure = pressure;
                pressure_msg.variance = 0.0;  // Variance can be set if known
                pressure_pub_->publish(pressure_msg);
                
                std::cout << "Published pressure: " << pressure << std::endl;
                // RCLCPP_INFO(this->get_logger(), "Published depth: %f", pressure);
            }
        }
        if (message.rfind("$BATTE", 0) == 0){
            double voltage, current;
            if(sscanf(message.c_str(), "$BATTE,%lf,%lf", &voltage, &current) == 2){
                sensor_msgs::msg::BatteryState battery_msg;
                battery_msg.voltage = voltage;
                battery_msg.current = current;

                battery_pub_->publish(battery_msg);
            }
        }
        if (message.rfind("$LEAK", 0) == 0) {
            int leak;
            if (sscanf(message.c_str(), "$LEAK,%d", &leak) == 1) {
                sensor_msgs::msg::FluidPressure leak_msg;
                leak_msg.header.stamp = this->now();
                leak_msg.fluid_pressure = leak;
                leak_pub_->publish(leak_msg);
            }
        }
        // Add processing for other message types (BATTE, LEAK) here if needed
    }


    std::string buffer_;
    const char delimiter_ = '\n';

    rclcpp::Subscription<frost_interfaces::msg::UCommand>::SharedPtr control_command_sub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr leak_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct sp_port *serial_port_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
