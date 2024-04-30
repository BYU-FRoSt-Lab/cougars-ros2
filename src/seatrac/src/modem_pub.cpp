#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/commands.h>
#include <seatrac_driver/messages/Messages.h>

#include "rclcpp/rclcpp.hpp"
#include "frost_interfaces/msg/modem_rec.hpp"
#include "frost_interfaces/msg/modem_send.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace narval::seatrac;

/*
Questions to ask Nelson: 
1. how to run in ros
2. how to integrate in project
3. how to log errors
*/


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
  }

  // this method is called on any message returned by the beacon.
  // it copies the modem data to a ros message of type ModemRec
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
        //Maybe just error log it?
      } break;

      case CID_STATUS:
        //TODO: determine if we want to send status msgs over ros
              //they could potentially include some calibration info
        break;
    }
  }

private:

  rclcpp::Publisher<frost_interfaces::msg::ModemRec>::SharedPtr publisher_;
  rclcpp::Subscription<frost_interfaces::msg::ModemSend>::SharedPtr subscriber_;

  size_t count_;

  //recieves command to modem from the ModemRec topic and sends the command
  // to the modem
  void modem_send_callback(const frost_interfaces::msg::ModemSend::SharedPtr rosmsg) {
    //TODO: add type casts as necessary
    CID_E msgId = rosmsg.msgId;
    switch(rosmsg.msgId) {
      default: break; //TODO: print bad id error.
      case CID_DAT_SEND: {
        messages::DataSend message; //struct contains message to send to modem

        message.destId    = rosmsg.destId;
        message.msgType   = rosmsg.msgType;
        message.packetLen = std::min(rosmsg.packetLen, 31);
        //TODO: add log report of modem message sent

        std::memcpy(message.packetData, rosmsg.packetData, message.packetLen);
        this->send(sizeof(message), (const uint8_t*)&message);

      } break;

      case CID_PING_SEND: {
        messages::PingSend::Request req;
        req.target    = rosmsg.destId;
        req.pingType  = rosmsg.msgType;
        this->send(sizeof(req), (const uint8_t*)&req);
      } break;
      //add case for calibration
    }
  }

  //copies the fields from the acofix struct into the ModemRec ros message
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
