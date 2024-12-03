
#include "cougars_coms/coms_protocol.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"

#include <iostream>
#include <chrono>
#include <memory>

using namespace std::literals::chrono_literals;

using std::placeholders::_1;
using namespace cougars_coms;

class ComsNode : public rclcpp::Node {
public:
    ComsNode() : Node("cougars_coms") {
        this->modem_subscriber = this->create_subscription<seatrac_interfaces::msg::ModemRec>(
            "modem_rec", 10,
            std::bind(&ComsNode::listen_to_modem, this, _1)
        );

        this->thruster_arming_client = this->create_client<std_srvs::srv::SetBool>(
            "arm_thruster"
        );
    }

    void listen_to_modem(seatrac_interfaces::msg::ModemRec msg) {
        COUG_MSG_ID id = (COUG_MSG_ID)msg.packet_data[0];
        switch(id) {
            default: break;
            case EMPTY: break;
            case EMERGENCY_KILL: {
                bool success = kill_thruster();
            } break;
        }
    }

    bool kill_thruster() {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        while (!this->thruster_arming_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                    "Interrupted while waiting for the arm_thruster service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "arm_thruster service not available, waiting again...");
        }
        auto result = this->thruster_arming_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "thruster has been deactivated.");
            return true;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service arm_thruster");
            return false;
        }
    }


private:

    rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_subscriber;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr thruster_arming_client;

};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ComsNode>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}