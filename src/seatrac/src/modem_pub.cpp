#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/commands.h>
#include <seatrac_driver/messages/Messages.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace narval::seatrac;

// The class needs to inherit from both the ROS node and driver classes
class ModemDataPublisher : public rclcpp::Node, public SeatracDriver {
public:
  ModemDataPublisher()
      : Node("modem_data_publisher"), SeatracDriver("/dev/ttyUSB0"), count_(0) {
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("modem_data", 10);
    timer_ = this->create_wall_timer(
        5000ms, std::bind(&ModemDataPublisher::timer_callback, this));
    this->ping_beacon(BEACON_ID_10);
  }

  void ping_beacon(BID_E target, AMSGTYPE_E pingType = MSG_REQU) {
    messages::PingSend::Request req;
    req.target = target;
    req.pingType = pingType;
    this->send(sizeof(req), (const uint8_t *)&req);
  }

  // this method is called on any message returned by the beacon.
  void on_message(CID_E msgId, const std::vector<uint8_t> &data) {
    switch (msgId) {
    default:
      break;
    case CID_PING_ERROR: {
      messages::PingError response;
      response = data;

      auto message = std_msgs::msg::String();
      message.data = "ERROR: No modem ping response detected";
      RCLCPP_INFO(this->get_logger(), message.data.c_str());
      publisher_->publish(message);

      this->ping_beacon(response.beaconId);
    } break;
    case CID_PING_RESP: {
      messages::PingResp response;
      response = data;

      auto message = std_msgs::msg::String();
      message.data = "TODO: Add modem ping message type here";
      RCLCPP_INFO(this->get_logger(), message.data.c_str());
      publisher_->publish(message);

      this->ping_beacon(response.acoFix.srcId);
    } break;
    case CID_STATUS:
      break;
    }
  }

private:
  void timer_callback() {
    // Code here will execute every 5000ms
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModemDataPublisher>());
  rclcpp::shutdown();
  return 0;
}