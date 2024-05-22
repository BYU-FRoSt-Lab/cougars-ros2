#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/commands.h>
#include <seatrac_driver/messages/Messages.h>

#include "rclcpp/rclcpp.hpp"
#include "frost_interfaces/msg/modem_rec.hpp"
#include "frost_interfaces/msg/modem_send.hpp"
#include "frost_interfaces/srv/emergency_stop.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace narval::seatrac;

//TODO: maybe add commandline arg for serial port

// The class needs to inherit from both the ROS node and driver classes
class ModemRosNode : public rclcpp::Node, public SeatracDriver {
public:
  ModemRosNode()
      : Node("modem_ros_node"), SeatracDriver("/dev/ttyUSB0"), count_(0) {
    RCLCPP_INFO(this->get_logger(), "Starting seatrac modem Node");
    publisher_ =
        this->create_publisher<frost_interfaces::msg::ModemRec>("modem_rec", 10);
    subscriber_ = 
        this->create_subscription<frost_interfaces::msg::ModemSend>("modem_send", 10,
                      std::bind(&ModemRosNode::modem_send_callback, this, _1));
    emergency_stop_client_ = 
        this->create_client<frost_interfaces::srv::EmergencyStop>("emergency_stop", 10);
    
  }

  // this method is called on any message returned by the beacon.
  // it copies the modem data to a ros message of type ModemRec
  void on_message(CID_E msgId, const std::vector<uint8_t> &data) {
    switch (msgId) {
      default: {
        RCLCPP_INFO(this->get_logger(), "Received unknown message from seatrac modem. msgId: %d", msgId); 
      } break;
      case CID_DAT_RECEIVE: {
        messages::DataReceive response;     //struct that contains response fields
        response = data;                    //operator overload fills in response struct with correct data

        auto msg = frost_interfaces::msg::ModemRec();
        msg.msg_id = CID_DAT_RECEIVE;
        msg.packet_len = response.packetLen;
        std::memcpy(&msg.packet_data, response.packetData, response.packetLen);
        cpyFixtoRosmsg(msg, response.acoFix);

        //check for emergency stop
        if (response.packetLen >= 4
            && response.packetData[0] == 'S'
            && response.packetData[1] == 'T'
            && response.packetData[2] == 'O'
            && response.packetData[3] == 'P'
        ) {
          execute_emergency_stop(response);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing ModemRec CID_DAT_RECEIVE");
        publisher_->publish(msg);
      } break;
      case CID_DAT_ERROR: {
        messages::DataError response;
        response = data;
        std::ostringstream err;
        err << "Error with seatrac modem data message." << std::endl << response;
        RCLCPP_ERROR(this->get_logger(), err.str());
      } break;

      case CID_PING_RESP: {
        messages::PingResp response;
        response = data;

        auto msg = frost_interfaces::msg::ModemRec();
        msg.msg_id = CID_PING_RESP;
        msg.packet_len = 0;
        cpyFixtoRosmsg(msg, response.acoFix);

        RCLCPP_INFO(this->get_logger(), "Publishing ModemRec CID_PING_RESP");
        publisher_->publish(msg);
      } break;
      case CID_PING_ERROR: {
        messages::PingError response;
        response = data;
        std::ostringstream err;
        err << "Error with seatrac modem ping message." << std::endl << response;
        RCLCPP_ERROR(this->get_logger(), err.str()); //TODO: add response diagnostic data to message
      } break;

      case CID_STATUS:
        //Too many status messages so bypasing display
        break;
    }
  }

private:

  rclcpp::Publisher<frost_interfaces::msg::ModemRec>::SharedPtr publisher_;
  rclcpp::Subscription<frost_interfaces::msg::ModemSend>::SharedPtr subscriber_;
  rclcpp::Client<frost_interfaces::srv::EmergencyStop>::SharedPtr emergency_stop_client_;

  size_t count_;

  // recieves command to modem from the ModemRec topic and sends the command
  // to the modem
  void modem_send_callback(const frost_interfaces::msg::ModemSend::SharedPtr rosmsg) {
    CID_E msgId = static_cast<CID_E>(rosmsg->msg_id);
    switch(msgId) {
      default: {
        RCLCPP_ERROR(this->get_logger(), "Unsupported seatrac message id for broadcasting messages: %d", msgId);
      } break;
      case CID_DAT_SEND: {
        messages::DataSend::Request req; //struct contains message to send to modem

        req.destId    = static_cast<BID_E>(rosmsg->dest_id);
        req.msgType   = static_cast<AMSGTYPE_E>(rosmsg->msg_type);
        req.packetLen = std::min(rosmsg->packet_len, (uint8_t)31);
        
        std::memcpy(req.packetData, rosmsg->packet_data.data(), req.packetLen);
        RCLCPP_INFO(this->get_logger(), "Seatrac modem broadcasting CID_DAT_SEND message. String is '%s'", req.packetData);
        this->send(sizeof(req), (const uint8_t*)&req);

      } break;

      case CID_PING_SEND: {
        messages::PingSend::Request req;
        req.target    = static_cast<BID_E>(rosmsg->dest_id);
        req.pingType  = static_cast<AMSGTYPE_E>(rosmsg->msg_type);
        RCLCPP_INFO(this->get_logger(), "Seatrac modem broadcasting CID_DAT_SEND message");
        this->send(sizeof(req), (const uint8_t*)&req);
      } break;
      //TODO: add case for calibration
    }
  }

  //copies the fields from the acofix struct into the ModemRec ros message
  inline void cpyFixtoRosmsg(frost_interfaces::msg::ModemRec& msg, ACOFIX_T& acoFix) {
    msg.attitude_yaw = acoFix.attitudeYaw;
    msg.attitude_pitch = acoFix.attitudePitch;
    msg.attitude_roll = acoFix.attitudeRoll;
    msg.depth_local = acoFix.depthLocal;
    msg.vos = acoFix.vos;
    msg.rssi = acoFix.rssi;

    msg.range_valid = (acoFix.flags & 0x1)? true:false;
    msg.usbl_valid = (acoFix.flags & 0x2)? true:false;
    msg.position_valid = (acoFix.flags & 0x4)? true:false;

    if(msg.range_valid) {
      msg.range_count = acoFix.range.count;
      msg.range_time = acoFix.range.time;
      msg.range_dist = acoFix.range.dist;
    }
    if(msg.usbl_valid) {
      msg.usbl_channels = acoFix.usbl.channelCount;
      std::memcpy(&msg.usbl_rssi, acoFix.usbl.rssi, acoFix.usbl.channelCount);
      msg.usbl_azimuth = acoFix.usbl.azimuth;
      msg.usbl_elevation = acoFix.usbl.elevation;
      msg.usbl_fit_error = acoFix.usbl.fitError;
    }
    if(msg.position_valid) {
      msg.position_easting = acoFix.position.easting;
      msg.position_northing = acoFix.position.northing;
      msg.position_depth = acoFix.position.depth;
    }
  }

  inline void execute_emergency_stop(messages::DataRecieve& response) {
    auto stop_request = std::make_shared<frost_interfaces::srv::EmergencyStop::Request>();
    std::ostringstream err;
    err << "Recieved STOP signal from beacon. Message Details: " << std::endl << response;
    stop_request.error = err.str();
    auto result = client->async_send_request(request);
    if(rclcpp::spin_until_future_complete(this, result) == 
        rclcpp::FutureReturnCode::SUCCESS)
    {
      char confirmation_text[] = "STOPPED";
      commands::data_send(
          this,
          MSG_OWAY,
          sizeof(confirmation_text),
          confirmation_text
      );
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(), "Failed to call Emergency Stop service");
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModemRosNode>());
  rclcpp::shutdown();
  return 0;
}





