

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "cougars_coms/coms_protocol.hpp"

#include <iostream>
#include <chrono>
#include <memory>

using namespace std::literals::chrono_literals;
using std::placeholders::_1;
using namespace cougars_coms;

class ComsNode : public rclcpp::Node {
public:
    ComsNode() : Node("cougars_coms") {

        this->declare_parameter<int>("base_station_beacon_id", 15);

        this->base_station_beacon_id_ = this->get_parameter("base_station_beacon_id").as_int();

        this->modem_subscriber_ = this->create_subscription<seatrac_interfaces::msg::ModemRec>(
            "modem_rec", 10,
            std::bind(&ComsNode::listen_to_modem, this, _1)
        );

        this->dvl_dead_reckoning = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "dvl/dead_reckoning", 10,
            std::bind(&ComsNode::listen_to_pose, this, _1)
        );

        this->modem_publisher_ = this->create_publisher<seatrac_interfaces::msg::ModemSend>("modem_send", 10);

        this->thruster_client_ = this->create_client<std_srvs::srv::SetBool>(
            "arm_thruster"
        );

        this->surface_client_ = this->create_client<std_srvs::srv::SetBool>(
            "surface"
        );

        this->init_coug_client_ = this->create_client<std_srvs::srv::SetBool>(
            "init_coug"
        );
    }

    void listen_to_modem(seatrac_interfaces::msg::ModemRec msg) {
        COUG_MSG_ID id = (COUG_MSG_ID)msg.packet_data[0];
        switch(id) {
            default: break;
            case EMPTY: break;
            case EMERGENCY_SURFACE: {
                emergency_surface();
            } break;
            case EMERGENCY_KILL: {
                kill_thruster();
            } break;
            case INIT_COUG: {
                init_coug();
            } break;
            case REQUEST_STATUS: {
                send_status();
            } break;
        }
    }

    void listen_to_pose(geometry_msgs::msg::PoseWithCovarianceStamped msg){
        this->x = msg.pose.pose.position.x;
        this->y = msg.pose.pose.position.y;
    }

    void kill_thruster() {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;
        while (!this->thruster_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                    "Interrupted while waiting for the arm_thruster service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "arm_thruster service not available, waiting again...");
        }

        auto result_future = this->thruster_client_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response_future) {
                try {
                    auto response = response_future.get();
                    if (response->success) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Thruster has been deactivated.");
                    } else {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to deactivate thruster.");
                    }
                    ConfirmEmergencyKill msg;
                    msg.success = response->success;
                    this->send_acoustic_message(base_station_beacon_id_, 1, (uint8_t*)&msg);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Error while trying to deactivate thruster: %s", e.what());
                }
            }
        );
    }

    void emergency_surface() {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        while (!this->surface_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                    "Interrupted while waiting for the surface service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "surface service not available, waiting again...");
        }

        auto result_future = this->surface_client_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response_future) {
                try {
                    auto response = response_future.get();
                    if (response->success) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Surface command was successfull.");
                    } else {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Surface command failed.");
                    }
                    ConfirmEmergencySurface msg;
                    msg.success = response->success;
                    this->send_acoustic_message(base_station_beacon_id_, 1, (uint8_t*)&msg);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Error while trying to send surface command: %s", e.what());
                }
            }
        );
    }

    void init_coug() {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        while (!this->init_coug_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                    "Interrupted while waiting for the init_coug service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "init_controls service not available, waiting again...");
        }

        auto result_future = this->init_coug_client_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response_future) {
                try {
                    auto response = response_future.get();
                    if (response->success) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coug has been initialized.");
                    } else {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize Coug");
                    }
                    ConfirmInitCoug msg;
                    msg.success = response->success;
                    this->send_acoustic_message(base_station_beacon_id_, 1, (uint8_t*)&msg);
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Error while trying to initialize coug: %s", e.what());
                }
            }
        );
    }

    void send_status(){
        VehicleStatus status_msg;
        status_msg.moos_waypoint = 0;
        status_msg.moos_behavior_number = 0;
        status_msg.x = this->x;
        status_msg.y = this->y;
        status_msg.depth = 0;
        status_msg.heading = 0;
        int target_id = 0;
        send_acoustic_message(target_id, sizeof(status_msg), (uint8_t*)&status_msg);
    }

    void send_acoustic_message(int target_id, int message_len, uint8_t* message) {
        auto request = seatrac_interfaces::msg::ModemSend();
        request.msg_id = 0x60; //CID_DAT_SEND
        request.dest_id = (uint8_t)target_id;
        request.msg_type = 0x0; //MSG_OWAY, data sent one way without response or position data
        request.packet_len = (uint8_t)std::min(message_len, 31);
        std::memcpy(&request.packet_data, message, request.packet_len);
        
        this->modem_publisher_->publish(request);
    }

private:

    rclcpp::Subscription<seatrac_interfaces::msg::ModemRec>::SharedPtr modem_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr dvl_dead_reckoning;
    rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_publisher_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr thruster_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr surface_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr init_controls_client_;

    float x;
    float y;
    int base_station_beacon_id_;

};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ComsNode>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}