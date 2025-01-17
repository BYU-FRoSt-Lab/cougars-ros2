#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>
#include <memory>
#include <string>


// ros2 stuff
#include "frost_interfaces/msg/desired_depth.hpp"
#include "frost_interfaces/msg/desired_heading.hpp"
#include "frost_interfaces/msg/desired_speed.hpp"
// #include "frost_interfaces/msg/vehicle_status.hpp"
#include "seatrac_interfaces/msg/modem_status.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// MOOS stuff
// https://gobysoft.org/doc/moos/class_c_m_o_o_s_msg.html
#include "MOOS/libMOOS/Comms/MOOSAsyncCommClient.h"
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "MOOS/libMOOS/Comms/MOOSMsg.h"
#include "MOOS/libMOOS/Utils/CommandLineParser.h"
#include <exception>

#include <cstdlib>
#include <iostream>

#define MOOS_MISSION_DIR "/home/frostlab/ros2_ws/moos_tools/"

using std::placeholders::_1;

MOOS::MOOSAsyncCommClient Comms;
typedef std::vector<CMOOSMsg> MsgVector;
typedef std::vector<MOOS::ClientCommsStatus> CommsStatusVector;

// global for moos and ros to be able to use


rclcpp::Publisher<frost_interfaces::msg::DesiredDepth>::SharedPtr
    desired_depth_publisher_;
rclcpp::Publisher<frost_interfaces::msg::DesiredHeading>::SharedPtr
    desired_heading_publisher_;
rclcpp::Publisher<frost_interfaces::msg::DesiredSpeed>::SharedPtr
    desired_speed_publisher_;

// MOOS functions

class MOOSBridge : public rclcpp::Node {
public:
  MOOSBridge() : Node("moos_bridge") {

    if (this->get_parameter("gps").as_string() == "true"){
      subscription_vehicle_status_ =
          this->create_subscription<nav_msgs::msg::Odometry>(
              "gps_odom", 10,
              std::bind(&MOOSBridge::ros_vehicle_status_listener, this, _1));
    }
    else{
      // vehicle status listener from the factor graph filter
      subscription_vehicle_status_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
          "smoothed_output", 10,
          std::bind(&MOOSBridge::ros_vehicle_status_listener, this, _1));
      }
    // just grab the heading straight from the modem for now, there is a converter in cougars_sim
    actual_heading_subscription_ =
        this->create_subscription<seatrac_interfaces::msg::ModemStatus>(
            "modem_status", 10,
            std::bind(&MOOSBridge::actual_heading_callback, this, _1));
    // publishers
    desired_depth_publisher_ =
        this->create_publisher<frost_interfaces::msg::DesiredDepth>(
            "desired_depth", 10);
    desired_heading_publisher_ =
        this->create_publisher<frost_interfaces::msg::DesiredHeading>(
            "desired_heading", 10);
    desired_speed_publisher_ =
        this->create_publisher<frost_interfaces::msg::DesiredSpeed>(
            "desired_speed", 10);
  }

private:
  // needs to listen to (x,y), depth, speed,
  // heading -->  NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING, NAV_DEPTH
  void
  ros_vehicle_status_listener(const nav_msgs::msg::Odometry &msg) {

    double nav_x, nav_y, nav_depth, nav_speed;


    if(this->get_parameter("sim").as_string() == "true"){

      nav_x = -msg.pose.pose.position.y;
      nav_y = msg.pose.pose.position.x;

    }
    else{

      nav_x = msg.pose.pose.position.x;
      nav_y = msg.pose.pose.position.y;

    }
    // nav_depth = msg.pose.pose.position.z;
    // nav_speed = msg.coug_odom.twist.twist.linear.x;

    nav_speed = 20.0;

    

    // publish to MOOS-IvP
    Comms.Notify("NAV_X", nav_x);
    Comms.Notify("NAV_Y", nav_y);
    // Comms.Notify("NAV_DEPTH", nav_depth);
    Comms.Notify("NAV_SPEED", nav_speed);
    
  }


  void
  actual_heading_callback(const seatrac_interfaces::msg::ModemStatus &msg) {
      //Heading is in degrees east of true north between -180 and 180
      //TODO: make sure this is what we want 
      // (Note: MOOS defines yaw to be negative heading)
                  // yaw comes in -180 to 180 (degrees)

    double nav_heading;
    
    if (msg.attitude_yaw < 0.0) {
      nav_heading = 360.0 + (0.1 * msg.attitude_yaw);
    } else {
      nav_heading = (0.1 * msg.attitude_yaw);
    }

    Comms.Notify("NAV_HEADING", nav_heading);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_vehicle_status_;
  rclcpp::Subscription<seatrac_interfaces::msg::ModemStatus>::SharedPtr actual_heading_subscription_;
};



bool OnConnect(void *pParam) {
  CMOOSCommClient *pC = reinterpret_cast<CMOOSCommClient *>(pParam);
  // from MOOS-IVP to ros base
  pC->Register("DESIRED_SPEED", 0.0);
  pC->Register("DESIRED_HEADING", 0.0);
  pC->Register("DESIRED_DEPTH", 0.0);
  pC->Register("NAV_SPEED", 0.0);
  pC->Register("NAV_HEADING", 0.0);
  pC->Register("NAV_DEPTH", 0.0);

  // std::string command = "uPokeDB " + MOOS_MISSION_DIR + "coug.moos" + " " +
  // variable + "=" + value " , MOOS_MANUAL_OVERIDE=false"; int result =
  // system(command.c_str());
  return 0;
}

void PublishDesiredValue(double value, std::string key) {

  if (key == "DESIRED_SPEED") {
    auto message = frost_interfaces::msg::DesiredSpeed();
    message.desired_speed = value;
    desired_speed_publisher_->publish(message);
  } else if (key == "DESIRED_HEADING") {
    auto message = frost_interfaces::msg::DesiredHeading();
    message.desired_heading = 90 - value;
    if (message.desired_heading < -180.0) {
      message.desired_heading = message.desired_heading + 360;
    }
    desired_heading_publisher_->publish(message);
  } else if (key == "DESIRED_DEPTH") {
    auto message = frost_interfaces::msg::DesiredDepth();
    message.desired_depth = value;
    desired_depth_publisher_->publish(message);
  }

  
}


// Receives the moos stuff
bool OnMail(void *pParam) {
  CMOOSCommClient *pC = reinterpret_cast<CMOOSCommClient *>(pParam);
  MOOSMSG_LIST M;
  pC->Fetch(M);
  MOOSMSG_LIST::iterator q;
  for (q = M.begin(); q != M.end(); q++) {
    CMOOSMsg &msg = *q;
    std::string key = msg.GetKey();
    double value = msg.GetDouble();
    PublishDesiredValue(value, key);
    std::cout << key << ": " << value << std::endl;

    // if you want to print all the values registered for, then uncomment this
    // q->Trace();
  }
  return true;
}

int main(int argc, char *argv[]) {

  // MOOS stuff
  Comms.SetOnMailCallBack(OnMail, &Comms);
  Comms.SetOnConnectCallBack(OnConnect, &Comms);
  Comms.Run("localhost", 9000, "my_connection");

  // ROS2 stuff
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MOOSBridge>());
  rclcpp::shutdown();

  return 0;
}