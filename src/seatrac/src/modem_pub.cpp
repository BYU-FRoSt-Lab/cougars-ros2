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
#include "frost_interfaces/msg/ModemSend.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace narval::seatrac;

//TODO: change name to represent that it is both a publisher and subscriber
// figured it'd probably be easier to have one serial connection to the modem

// The class needs to inherit from both the ROS node and driver classes
class ModemDataPublisher : public rclcpp::Node, public SeatracDriver {
public:
  ModemDataPublisher()
      : Node("modem_data_publisher"), SeatracDriver("/dev/ttyUSB0"), count_(0) {
    publisher_ =
        this->create_publisher<frost_interfaces::msg::ModemRec>("modem_rec", 10);
    subscriber_ = 
        this->create_subscription<frost_interfaces::msg::ModemSend>("modem_send", 10,
                      std::bind(&ModemDataPublisher::modem_send_callback, this, _1));


    // timer_ = this->create_wall_timer( //TODO: may not need timer
    //     5000ms, std::bind(&ModemDataPublisher::timer_callback, this));
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
        std::memcpy(msg.packetData, response.packetData, response.packetLen);
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
        msg.packetLen = 0;
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

  //TODO: do we need timer_callback?
  // void timer_callback() {
  //   // Code here will execute every 5000ms
  // }
  // rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<frost_interfaces::msg::ModemRec>::SharedPtr publisher_;
  rclcpp::Subscription<frost_interfaces::msg::ModemSend>::SharedPtr subscriber_;

  size_t count_;

  //recieves command to modem from the ModemRec topic and sends the command
  // to the modem
  void modem_send_callback(const frost_interfaces::msg::ModemSend::SharedPtr msg) {
    //TODO: implement logic to send command to modem
  }

  //copies the fields from the acofix struct into the modem ros message
  inline void cpyFixtoRosmsg(frost_interfaces::msg::ModemRec& msg, ACOFIX_T& acofix) {
    msg.attitudeYaw = acoFix.attitudeYaw;
    msg.attitudePitch = acoFix.attitudePitch;
    msg.attitudeRoll = acoFix.attitudeRoll;
    msg.depthLocal = acoFix.depthLocal;
    msg.VOS = acoFix.vos;
    msg.rssi = acoFix.rssi;

    msg.rangeValid = (acoFix.flags & 0x1)? true:false;
    msg.usblValid = (acoFix.flags & 0x2)? true:false;
    msg.positionValid = (acoFix.flags & 0x4)? true:false;

    if(msg.rangeValid) {
      msg.rangeCount = acoFix.range.count;
      msg.rangeTime = acoFix.range.time;
      msg.rangeDist = acoFix.range.dist;
    }
    if(msg.usblValid) {
      msg.usblChannels = acoFix.usbl.channelCount;
      std::memcpy(msg.usblRSSI, acoFix.usbl.rssi, acoFix.usbl.channelCount);
      msg.usblAzimuth = acoFix.usbl.azimuth;
      msg.usblElecation = acoFix.usbl.elevation;
      msg.usblFitError = acoFix.usbl.fitError;
    }
    if(msg.positionValid) {
      msg.positionEasting = acoFix.position.easting;
      msg.positionNorthing = acoFix.position.positionNorthing;
      msg.positionDepth = acoFix.position.depth;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModemDataPublisher>());
  rclcpp::shutdown();
  return 0;
}