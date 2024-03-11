#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/commands.h>
#include <seatrac_driver/messages/Messages.h>

#include "rclcpp/rclcpp.hpp"
#include "frost_interfaces/msg/ModemRec.hpp"

using namespace std::chrono_literals;
using namespace narval::seatrac;

// The class needs to inherit from both the ROS node and driver classes
class ModemDataPublisher : public rclcpp::Node, public SeatracDriver {
private:
  //copies the fields from the acofix struct into the modem ros message
  void cpyFixtoRosmsg(frost_interfaces::msg::ModemRec& msg, ACOFIX_T& acofix) {
    //TODO: add code to copy acofix into msg
  }

public:
  ModemDataPublisher()
      : Node("modem_data_publisher"), SeatracDriver("/dev/ttyUSB0"), count_(0) {
    publisher_ =
        this->create_publisher<frost_interfaces::msg::ModemRec>("modem_rec", 10);
    timer_ = this->create_wall_timer(
        5000ms, std::bind(&ModemDataPublisher::timer_callback, this));
  }

  // this method is called on any message returned by the beacon.
  void on_message(CID_E msgId, const std::vector<uint8_t> &data) {
    switch (msgId) {
      case CID_DAT_RECEIVE: {
        messages::DataReceive response;     //struct that contains response fields
        response = data;                    //operator overload fills in response struct with correct data

        auto msg = frost_interfaces::msg::ModemRec();
        msg.msgId = CID_DAT_RECEIVE;
        msg.packetLen = response.packetLen;
        msg.packetData = response.packetData; //TODO: make sure this syntax to copy lists works
        cpyFixtoRosmsg(msg, response.acoFix);
        //TODO: add rclcpp info log here? not sure how to implement
        publisher_->publish(msg);
      } break;
      case CID_DAT_ERROR: {
        messages::DataError response;
        response = data;
        //TODO: find out how/if to send error msgs over ros
      } break;

      case CID_PING_RESP: {
        messages::PingResp response;
        response = data;

        auto msg = frost_interfaces::msg::ModemRec();
        msg.msgId = CID_PING_RESP;
        cpyFixtoRosmsg(msg, response.acoFix);

        publisher_->publish(msg);
      } break;
      case CID_PING_ERROR: {
        messages::PingError response;
        response = data;
        //TODO: find out how/if to send error msgs over ros
      } break;

      case CID_STATUS:
        //TODO: determine if we want to send status msgs over ros
              //they could potentially include some calibration info
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