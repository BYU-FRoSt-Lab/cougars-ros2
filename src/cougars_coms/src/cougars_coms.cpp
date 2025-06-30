



#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "dvl_msgs/msg/dvldr.hpp"
#include "frost_interfaces/msg/system_status.hpp"
#include "frost_interfaces/msg/localization_data.hpp"



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

        this->safety_subscriber_ = this->create_subscription<frost_interfaces::msg::SystemStatus>(
            "safety_status", 10,
            std::bind(&ComsNode::safety_callback, this, _1)
        );

        this->battery_subscriber_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery/data", 10,
            std::bind(&ComsNode::battery_callback, this, _1)
        );

        this->pressure_subscriber_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "pressure/data", 10,
            std::bind(&ComsNode::pressure_callback, this, _1)
        );

        this->depth_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "depth_data", 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped msg) {
                this->position_z = msg.pose.pose.position.z;
            }
        );
        

        this->modem_subscriber_ = this->create_subscription<seatrac_interfaces::msg::ModemRec>(
            "modem_rec", 10,
            std::bind(&ComsNode::listen_to_modem, this, _1)
        );

        this->modem_publisher_ = this->create_publisher<seatrac_interfaces::msg::ModemSend>("modem_send", 10);


        this->thruster_client_ = this->create_client<std_srvs::srv::SetBool>(
            "arm_thruster"
        );


        this->surface_client_ = this->create_client<std_srvs::srv::SetBool>(
            "surface"
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
            case REQUEST_STATUS: {
                send_status();
            } break;
            case REQUEST_LOCALIZATION_INFO: {
               send_localization_info(msg.src_id);
            } break;
            case LOCALIZATION_INFO: {
               record_localization_info(msg);
            } break;

        }
    }

    void battery_callback(sensor_msgs::msg::BatteryState msg) {
        // Here you can handle the battery data if needed
        this->battery_voltage = msg.percentage * 100; 
    }

    void safety_callback(frost_interfaces::msg::SystemStatus msg) {
        // Here you can handle the safety system status data if needed
        this->safety_mask =
            (msg.depth_status.data      ? 1 << 0 : 0) |
            (msg.gps_status.data        ? 1 << 1 : 0) |
            (msg.modem_status.data      ? 1 << 2 : 0) |
            (msg.dvl_status.data        ? 1 << 3 : 0) |
            (msg.emergency_status.data  ? 1 << 4 : 0);
    }

    void pressure_callback(sensor_msgs::msg::FluidPressure msg) {
        this->pressure = msg.fluid_pressure; 
    }

    void smoothed_odom_callback(nav_msgs::msg::Odometry msg) {
        this->position_x = msg.pose.pose.position.x;
        this->position_y = msg.pose.pose.position.y;
        this->velocity_x = msg.twist.twist.linear.x;
        this->velocity_y = msg.twist.twist.linear.y;
    }

    void depth_callback(geometry_msgs::msg::PoseWithCovarianceStamped msg) {
        this->depth = msg.pose.pose.position.z;
    }


    void kill_thruster() {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received kill command from base station");
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
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received surface command from base station");
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

    void send_status(){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending status to base station.");
        VehicleStatus status_msg;
        status_msg.waypoint = 0;
        status_msg.battery_voltage = this->battery_voltage;
        status_msg.battery_percentage = this->battery_percentage;
        status_msg.safety_mask = this->safety_mask;
        status_msg.x = this->position_x;
        status_msg.y = this->position_y;
        status_msg.heading = 0;
        status_msg.depth = this->position_z; 
        status_msg.x_vel = this->velocity_x;
        status_msg.y_vel = this->velocity_y;
        status_msg.pressure = this->pressure;
        send_acoustic_message(base_station_beacon_id_, sizeof(status_msg), (uint8_t*)&status_msg);
    }

    void send_localization_info(int src_id) {
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending localization info.");
       LocalizationInfo message;
       message.x = this->position_x;
       message.y = this->position_y;
       message.z = this->position_z;
       message.roll = this->roll;
       message.pitch = this->pitch;
       message.yaw = this->yaw;
       message.depth = this->depth;
       send_acoustic_message(src_id, sizeof(message), (uint8_t*)&message);
   }


   void record_localization_info(seatrac_interfaces::msg::ModemRec msg) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received localization info.");

        if (msg.packet_len < sizeof(LocalizationInfo)) {
            RCLCPP_ERROR(this->get_logger(),
                "packet_data too small for LocalizationInfo! Got %u bytes, expected %zu",
                msg.packet_len, sizeof(LocalizationInfo));
            return;
        }   

        const LocalizationInfo* info = reinterpret_cast<const LocalizationInfo*>(msg.packet_data.data());
        frost_interfaces::msg::LocalizationData localization_data;
        localization_data.vehicle_id = msg.src_id;
        localization_data.x = info->x;
        localization_data.y = info->y;
        localization_data.z = info->z;
        localization_data.roll = info->roll;
        localization_data.pitch = info->pitch;
        localization_data.yaw = info->yaw;
        localization_data.depth = info->depth;


        localization_data_publisher_->publish(localization_data);

        RCLCPP_INFO(this->get_logger(), "Received localization info from vehicle ID %d", msg.src_id);
        RCLCPP_INFO(this->get_logger(), "Localization Info: (x, y, z): (%.2f, %.2f, %.2f), (roll, pitch, yaw): (%.2f, %.2f, %.2f), depth: %.2f",
            localization_data.x, localization_data.y, localization_data.z,
            localization_data.roll, localization_data.pitch, localization_data.yaw,
            localization_data.depth);
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<frost_interfaces::msg::SystemStatus>::SharedPtr safety_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_subscriber_;

    rclcpp::Publisher<frost_interfaces::msg::LocalizationData>::SharedPtr localization_data_publisher_;
    rclcpp::Publisher<seatrac_interfaces::msg::ModemSend>::SharedPtr modem_publisher_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr thruster_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr surface_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr init_controls_client_;



    uint8_t position_x;
    uint8_t position_y;
    uint8_t position_z;
    uint8_t velocity_x;
    uint8_t velocity_y;
    uint8_t base_station_beacon_id_;
    uint8_t safety_mask;
    uint8_t battery_voltage;
    uint8_t battery_percentage;
    uint8_t heading;
    uint8_t pressure;

    float dvl_position_x;
    float dvl_position_y;
    float dvl_position_z;
    float roll;
    float pitch;
    float yaw;
    float depth;

};




int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto coms_node = std::make_shared<ComsNode>();
  rclcpp::spin(coms_node);
  rclcpp::shutdown();
  return 0;
}



