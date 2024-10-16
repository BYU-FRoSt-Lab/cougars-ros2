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
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_status.hpp"
#include "seatrac_interfaces/msg/modem_cmd_update.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"

// Replace this with the serial port that your seatrac beacon is connected to.
#define SEATRAC_SERIAL_PORT "/dev/ttyUSB0"

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace narval::seatrac;

/**
 * @brief A driver node for the Seatrac x150 USBL beacon
 * @author Clayton Smith
 * @date September 2024
 * 
 * This node interfaces with the seatrac x150 beacon. It publishes modem_rec 
 * with information on acoustic transmissions, modem_status with regular 
 * beacon status updates, and modem_cmd_update with command status codes and 
 * error codes. It broadcasts an acoustic message when it recieves modem_send.
 * 
 * Code copied from 
 * https://bitbucket.org/frostlab/seatrac_driver/src/main/tools/ros2_seatrac_ws/seatrac/src/modem_ros_node.cpp
 * 
 * Subscribes:
 * - modem_send (seatrac_interfaces/msg/ModemSend)
 * Publishes:
 * - modem_rec (seatrac_interfaces/msg/ModemRec)
 * - modem_status (seatrac_interfaces/msg/ModemStatus)
 * - modem_cmd_update (seatrac_interfaces/msg/ModemCmdUpdate)
 */
class ModemRosNode : public rclcpp::Node, public SeatracDriver {
public:

  std::string get_serial_port() {
    this->declare_parameter("seatrac_serial_port", "/dev/ttyUSB0");
    return this->get_parameter("seatrac_serial_port").as_string();
  }

  ModemRosNode()
      : Node("modem_ros_node"), SeatracDriver(this->get_serial_port()), count_(0) {
    RCLCPP_INFO(this->get_logger(), "Starting seatrac modem Node");
    rec_pub_ =
        this->create_publisher<seatrac_interfaces::msg::ModemRec>("modem_rec", 10);
    status_pub_ =
        this->create_publisher<seatrac_interfaces::msg::ModemStatus>("modem_status", 10);
    cmd_update_pub_ =
        this->create_publisher<seatrac_interfaces::msg::ModemCmdUpdate>("modem_cmd_update", 10);
    
    subscriber_ = 
        this->create_subscription<seatrac_interfaces::msg::ModemSend>("modem_send", 10,
                      std::bind(&ModemRosNode::modem_send_callback, this, _1));

    /**
     * @param vehicle_ID
     *
     * The Coug-UV vehicle ID, used to set the beacon ID.
     * The beacon ID is used to address acoustic messages.
     * An integer between 1 and 15 inclusive.
     */
    this->declare_parameter("vehicle_ID", 1);

    /**
     * @param water_salinity_ppt
     *
     * The salinity of the surounding water in parts per ton (ppt). 
     * Used to determine the velocity of sound to find the distance 
     * between beacons. 0 ppt for fresh water, 35 ppt for salt water.
     */
    this->declare_parameter("water_salinity_ppt", 0);

    BID_E beaconId = (BID_E)(this->get_parameter("vehicle_ID").as_int());
    uint16_t salinity = (uint16_t)(this->get_parameter("water_salinity_ppt").as_int()*10);

    wait_for_alive(beaconId, salinity);
  }

  void wait_for_alive(BID_E beaconId, uint16_t salinity) {
    bool got_resp = false;
    while(!got_resp) {
      messages::SysAlive resp;
      CID_E msgId = CID_SYS_ALIVE;
      got_resp = this->send_request(1, (const uint8_t*)&msgId, &resp);
    }
    RCLCPP_INFO(this->get_logger(), "Seatrac Beacon Connected and Responding");

    //Set beacon ID once connected
    SETTINGS_T settings = command::settings_get(*this).settings;
    settings.xcvrBeaconId = beaconId;
    settings.envSalinity = salinity;
    settings.statusFlags = STATUS_MODE_10HZ;
    settings.status_output = ATTITUDE | AHRS_COMP_DATA;
    command::settings_set(*this, settings);
    command::settings_save(*this);    
    beacon_connected = true;
  }

  // this method is called on any message returned by the beacon.
  // it copies the modem data to a ros message of type ModemRec
  void on_message(CID_E msgId, const std::vector<uint8_t> &data) {
    auto timestamp = this->get_clock()->now();
    switch (msgId) {
      default: {
        //RCLCPP_INFO(this->get_logger(), "Received unknown message from seatrac modem. msgId: %d", msgId); 
      } break;
      case CID_DAT_RECEIVE: {
        auto msg = seatrac_interfaces::msg::ModemRec();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::DataReceive report;     //struct that contains report fields
        report = data;                    //operator overload fills in report struct with correct data
        msg.packet_len = report.packetLen;
        msg.local_flag = report.localFlag;
        std::memcpy(&msg.packet_data, report.packetData, report.packetLen);
        cpyFixtoRosmsg(msg, report.acoFix);
        rec_pub_->publish(msg);
      } break;

      case CID_DAT_ERROR: {
        auto msg = seatrac_interfaces::msg::ModemCmdUpdate();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::DataError report;
        report = data;
        msg.command_status_code = report.status;
        msg.target_id = report.beaconId;
        cmd_update_pub_->publish(msg);
        std::ostringstream ss;
        ss << "Acoustic DATA Error. Status Code = " << report.status << ", Target ID = " << report.beaconId;
        RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
      } break;

      case CID_DAT_SEND: {
        auto msg = seatrac_interfaces::msg::ModemCmdUpdate();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::DataSend report;
        report = data;
        msg.command_status_code = report.status;
        msg.target_id = report.beaconId;
        cmd_update_pub_->publish(msg);
      } break;

      case CID_ECHO_RESP: {
        auto msg = seatrac_interfaces::msg::ModemRec();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::EchoResp report;     //struct that contains report fields
        report = data;                    //operator overload fills in report struct with correct data
        msg.local_flag = true;
        msg.packet_len = report.packetLen;
        std::memcpy(&msg.packet_data, report.packetData, report.packetLen);
        cpyFixtoRosmsg(msg, report.acoFix);
        rec_pub_->publish(msg);
      } break;

      case CID_ECHO_REQ: {
        auto msg = seatrac_interfaces::msg::ModemRec();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::EchoReq report;
        report = data;
        msg.packet_len = report.packetLen;
        std::memcpy(&msg.packet_data, report.packetData, report.packetLen);
        cpyFixtoRosmsg(msg, report.acoFix);
        rec_pub_->publish(msg);
      } break;

      case CID_ECHO_ERROR: {
        auto msg = seatrac_interfaces::msg::ModemCmdUpdate();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::EchoError report;
        report = data;
        msg.command_status_code = report.status;
        msg.target_id = report.beaconId;
        cmd_update_pub_->publish(msg);
        std::ostringstream ss;
        ss << "Acoustic ECHO Error. Status Code = " << report.status << ", Target ID = " << report.beaconId;
        RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
      } break;

      case CID_ECHO_SEND: {
        auto msg = seatrac_interfaces::msg::ModemCmdUpdate();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::EchoSend report;
        report = data;
        msg.command_status_code = report.status;
        msg.target_id = report.beaconId;
        cmd_update_pub_->publish(msg);
      } break;

      case CID_PING_RESP: {
        auto msg = seatrac_interfaces::msg::ModemRec();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::PingResp report;
        report = data;
        msg.packet_len = 0;
        msg.local_flag = true; //Ping messages are not sniffed.
        cpyFixtoRosmsg(msg, report.acoFix);
        rec_pub_->publish(msg);
      } break;

      case CID_PING_REQ: {
        auto msg = seatrac_interfaces::msg::ModemRec();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::PingReq report;
        report = data;
        msg.packet_len = 0;
        msg.local_flag = true;
        cpyFixtoRosmsg(msg, report.acoFix);
        rec_pub_->publish(msg);
      } break;

      case CID_PING_ERROR: {
        auto msg = seatrac_interfaces::msg::ModemCmdUpdate();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::PingError report;
        report = data;
        msg.command_status_code = report.statusCode;
        msg.target_id = report.beaconId;
        cmd_update_pub_->publish(msg);
        std::ostringstream ss;
        ss << "Acoustic PING Error. Status Code = " << report.statusCode << ", Target ID = " << report.beaconId;
        RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
      } break;

      case CID_PING_SEND: {
        auto msg = seatrac_interfaces::msg::ModemCmdUpdate();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::PingSend report;
        report = data;
        msg.command_status_code = report.statusCode;
        msg.target_id = report.target;
        cmd_update_pub_->publish(msg);
      } break;

      case CID_NAV_QUERY_RESP: {
        auto msg = seatrac_interfaces::msg::ModemRec();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::NavQueryResp report;
        report = data;
        msg.local_flag = report.localFlag;
        cpyFixtoRosmsg(msg, report.acoFix);
        msg.includes_remote_depth    = (report.queryFlags & QRY_DEPTH)?    true:false;
        msg.includes_remote_supply   = (report.queryFlags & QRY_SUPPLY)?   true:false;
        msg.includes_remote_temp     = (report.queryFlags & QRY_TEMP)?     true:false;
        msg.includes_remote_attitude = (report.queryFlags & QRY_ATTITUDE)? true:false;
        if(msg.includes_remote_depth)  msg.remote_depth  = report.remoteDepth;
        if(msg.includes_remote_supply) msg.remote_supply = report.remoteSupply;
        if(msg.includes_remote_temp)   msg.remote_temp   = report.remoteTemp;
        if(msg.includes_remote_attitude) {
          msg.remote_yaw   = report.remoteYaw;
          msg.remote_pitch = report.remotePitch;
          msg.remote_roll  = report.remoteRoll;
        }
        if(report.queryFlags & QRY_DATA) {
          msg.packet_len = report.packetLen;
          std::memcpy(&msg.packet_data, report.packetData, report.packetLen);
        } else msg.packet_len = 0;
        rec_pub_->publish(msg);
      } break;

      case CID_NAV_QUERY_REQ: {
        auto msg = seatrac_interfaces::msg::ModemRec();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::NavQueryReq report;
        report = data;
        msg.local_flag = report.localFlag;
        msg.packet_len = report.packetLen;
        std::memcpy(&msg.packet_data, report.packetData, report.packetLen);
        cpyFixtoRosmsg(msg, report.acoFix);
        rec_pub_->publish(msg);
        //Note that the field report.queryFlags is not saved to the ros msg.
      } break;

      case CID_NAV_ERROR: {
        auto msg = seatrac_interfaces::msg::ModemCmdUpdate();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::NavError report;
        report = data;
        msg.command_status_code = report.statusCode;
        msg.target_id = report.beaconId;
        cmd_update_pub_->publish(msg);
        std::ostringstream ss;
        ss << "Acoustic NAV Error. Status Code = " << report.statusCode << ", Target ID = " << report.beaconId;
        RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
      } break;

      case CID_NAV_QUERY_SEND: {
        auto msg = seatrac_interfaces::msg::ModemCmdUpdate();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::NavQuerySend report;
        report = data;
        msg.command_status_code = report.status;
        msg.target_id = report.beaconId;
        cmd_update_pub_->publish(msg);
      } break;

      // Fields don't match the ros message types provided.
      // case CID_XCVR_RX_ERR: {
      //   auto msg = seatrac_interfaces::msg::ModemRec();
      //   msg.msg_id = msgId;
      //   messages::XcvrReceptionError report;
      //   report = data;
      //   msg.command_status_code = report.statusCode;
      //   cpyFixtoRosmsg(msg, report.acousticFix);
      // } break;

      case CID_STATUS: {
        auto msg = seatrac_interfaces::msg::ModemStatus();
        msg.header.stamp = timestamp;
        msg.msg_id = msgId;
        messages::Status report;
        report = data;
        msg.timestamp = report.timestamp;
        if(report.contentType & ENVIRONMENT) {
          msg.includes_env_fields = true;
          msg.supply_voltage = report.environment.envSupply;
          msg.env_temp = report.environment.envTemp;
          msg.env_pressure = report.environment.envPressure;
          msg.depth_local = report.environment.envDepth;
          msg.vos = report.environment.envVos;
        }
        if(report.contentType & ATTITUDE) {
          msg.includes_local_attitude = true;
          msg.attitude_yaw   = report.attitude.attYaw;
          msg.attitude_pitch = report.attitude.attPitch;
          msg.attitude_roll  = report.attitude.attRoll;
        }
        if(report.contentType & AHRS_COMP_DATA) {
          msg.includes_comp_ahrs = true;
          msg.acc_x = report.compensatedAHRS.ahrsCompAccX;
          msg.acc_y = report.compensatedAHRS.ahrsCompAccY;
          msg.acc_z = report.compensatedAHRS.ahrsCompAccZ;
          msg.mag_x = report.compensatedAHRS.ahrsCompMagX;
          msg.mag_y = report.compensatedAHRS.ahrsCompMagY;
          msg.mag_z = report.compensatedAHRS.ahrsCompMagZ;
          msg.gyro_x = report.compensatedAHRS.ahrsCompGyroX;
          msg.gyro_y = report.compensatedAHRS.ahrsCompGyroY;
          msg.gyro_z = report.compensatedAHRS.ahrsCompGyroZ;
        }
        status_pub_->publish(msg);
      } break;
    }

    //rec_pub_->publish(msg);
  }

private:

  rclcpp::Publisher<seatrac_interfaces::msg::ModemRec>::SharedPtr rec_pub_;
  rclcpp::Publisher<seatrac_interfaces::msg::ModemStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<seatrac_interfaces::msg::ModemCmdUpdate>::SharedPtr cmd_update_pub_;
  rclcpp::Subscription<seatrac_interfaces::msg::ModemSend>::SharedPtr subscriber_;

  size_t count_;

  bool beacon_connected = false;

  // recieves command to modem from the ModemRec topic and sends the command
  // to the modem
  void modem_send_callback(const seatrac_interfaces::msg::ModemSend::SharedPtr rosmsg) {
    if(!beacon_connected) return;
    CID_E msgId = static_cast<CID_E>(rosmsg->msg_id);
    switch(msgId) {
      default: {
        std::ostringstream ss;
        ss << "Unsupported seatrac message id for broadcasting messages: " << msgId;
        RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
      } break;
      
      case CID_DAT_SEND: {
        messages::DataSend::Request req; //struct contains message to send to modem

        req.destId    = static_cast<BID_E>(rosmsg->dest_id);
        req.msgType   = static_cast<AMSGTYPE_E>(rosmsg->msg_type);
        req.packetLen = std::min(rosmsg->packet_len, (uint8_t)sizeof(req.packetData));

        std::memcpy(req.packetData, rosmsg->packet_data.data(), req.packetLen);

        std::ostringstream ss;
        ss << "Transmitting Acoustic DATA Message. Target ID = " << req.destId;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        this->send(sizeof(req), (const uint8_t*)&req);

      } break;

      case CID_ECHO_SEND: {
        messages::EchoSend::Request req; //struct contains message to send to modem

        req.destId    = static_cast<BID_E>(rosmsg->dest_id);
        req.msgType   = static_cast<AMSGTYPE_E>(rosmsg->msg_type);
        req.packetLen = std::min(rosmsg->packet_len, (uint8_t)sizeof(req.packetData));

        std::ostringstream ss;
        ss << "Transmitting Acoustic ECHO Message. Target ID = " << req.destId;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        this->send(sizeof(req), (const uint8_t*)&req);

      } break;

      case CID_PING_SEND: {
        messages::PingSend::Request req;
        req.target    = static_cast<BID_E>(rosmsg->dest_id);
        req.pingType  = static_cast<AMSGTYPE_E>(rosmsg->msg_type);

        std::ostringstream ss;
        ss << "Transmitting Acoustic PING Message. Target ID = " << req.target;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        this->send(sizeof(req), (const uint8_t*)&req);
      } break;

      case CID_NAV_QUERY_SEND: {
        messages::NavQuerySend::Request req;
        req.destId     = static_cast<BID_E>(rosmsg->dest_id);
        req.queryFlags = static_cast<NAV_QUERY_E>(rosmsg->nav_query_flags);
        req.packetLen  = rosmsg->packet_len;
        std::memcpy(req.packetData, rosmsg->packet_data.data(), req.packetLen);

        std::ostringstream ss;
        ss << "Transmitting Acoustic NAV Message. Target ID = " << req.destId;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        this->send(sizeof(req), (const uint8_t*)&req);
      }
      
    }
  }

  //copies the fields from the acofix struct into the ModemRec ros message
  inline void cpyFixtoRosmsg(seatrac_interfaces::msg::ModemRec& msg, ACOFIX_T& acoFix) {

    msg.dest_id = acoFix.destId;
    msg.src_id  = acoFix.srcId;

    msg.attitude_yaw = acoFix.attitudeYaw;
    msg.attitude_pitch = acoFix.attitudePitch;
    msg.attitude_roll = acoFix.attitudeRoll;
    
    msg.depth_local = acoFix.depthLocal;
    msg.vos = acoFix.vos;
    msg.rssi = acoFix.rssi;


    msg.includes_range = (acoFix.flags & RANGE_VALID)? true:false;
    msg.includes_usbl = (acoFix.flags & USBL_VALID)? true:false;
    msg.includes_position = (acoFix.flags & POSITION_VALID)? true:false;

    msg.position_enhanced = (acoFix.flags & POSITION_ENHANCED)? true:false;
    msg.position_flt_error = (acoFix.flags & POSITION_FLT_ERROR)? true:false;

    if(msg.includes_range) {
      msg.range_count = acoFix.range.count;
      msg.range_time = acoFix.range.time;
      msg.range_dist = acoFix.range.dist;
    }
    if(msg.includes_usbl) {
      msg.usbl_channels = acoFix.usbl.channelCount;
      std::memcpy(&msg.usbl_rssi, acoFix.usbl.rssi, acoFix.usbl.channelCount);
      msg.usbl_azimuth = acoFix.usbl.azimuth;
      msg.usbl_elevation = acoFix.usbl.elevation;
      msg.usbl_fit_error = acoFix.usbl.fitError;
    }
    if(msg.includes_position) {
      msg.position_easting = acoFix.position.easting;
      msg.position_northing = acoFix.position.northing;
      msg.position_depth = acoFix.position.depth;
    }

    std::ostringstream ss;
    ss << "Received acoustic transmission from " << acoFix.srcId;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());

  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto modem_ros_node = std::make_shared<ModemRosNode>();
  rclcpp::spin(modem_ros_node);
  rclcpp::shutdown();
  return 0;
}
