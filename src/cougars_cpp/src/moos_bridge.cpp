#include <functional>
#include <memory>

// ros2 stuff
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "frost_interfaces/msg/desired_depth.hpp"
#include "frost_interfaces/msg/desired_heading.hpp"
#include "frost_interfaces/msg/desired_speed.hpp"


// MOOS stuff
#include "MOOS/libMOOS/Comms/MOOSAsyncCommClient.h"
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "MOOS/libMOOS/Comms/MOOSMsg.h"
#include "MOOS/libMOOS/Utils/CommandLineParser.h"
#include <exception>

using std::placeholders::_1;

MOOS::MOOSAsyncCommClient Comms;
typedef std::vector<CMOOSMsg> MsgVector;
typedef std::vector<MOOS::ClientCommsStatus> CommsStatusVector;

// global for moos and ros to be able to use

rclcpp::Publisher<frost_interfaces::msg::DesiredDepth>::SharedPtr desired_depth_publisher_;
rclcpp::Publisher<frost_interfaces::msg::DesiredHeading>::SharedPtr desired_heading_publisher_;
rclcpp::Publisher<frost_interfaces::msg::DesiredSpeed>::SharedPtr desired_speed_publisher_;

// MOOS functions

class MOOSBridge : public rclcpp::Node {
public:
  MOOSBridge() : Node("moos_bridge") {


    // ros listeners

    // TODO: change these to the correct topics and message types
    subscription_latlon = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MOOSBridge::ros_latlon_listener, this, _1));
    subscription_depth = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MOOSBridge::ros_depth_listener, this, _1));
    subscription_heading = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MOOSBridge::ros_heading_listener, this, _1));
    subscription_speed = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MOOSBridge::ros_speed_listener, this, _1));


    // publishers
    desired_depth_publisher_ = this->create_publisher<frost_interfaces::msg::DesiredDepth>("desired_depth", 10);
    desired_heading_publisher_ = this->create_publisher<frost_interfaces::msg::DesiredHeading>("desired_heading", 10);
    desired_speed_publisher_ = this->create_publisher<frost_interfaces::msg::DesiredSpeed>("desired_speed", 10);
  }



private:

  // needs to listen to current latitude and longitude (x,y), depth, speed, heading -->  NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING, NAV_DEPTH
  void ros_laton_listener(const std_msgs::msg::String &msg) {
    // publish

    // TODO: extract message and get the gps lat lon and then convert to x, y

    /////////////////////////////////////////////////////////

    // put conversion code in here
    // get rid of dummy values 149.9 down below



    /////////////////////////////////////////////////////////



    Comms.Notify("NAV_X", 149.9);
    Comms.Notify("NAV_Y", 149.9);
  }
  void ros_depth_listener(const std_msgs::msg::String &msg) {
    // TODO: extract message

    /////////////////////////////////////////////////////////

  



    /////////////////////////////////////////////////////////





    // publish
    Comms.Notify("NAV_DEPTH", 149.9);
  }
  void ros_heading_listener(const std_msgs::msg::String &msg) {
    // TODO: extract message

    /////////////////////////////////////////////////////////

  



    /////////////////////////////////////////////////////////

    // publish
    Comms.Notify("NAV_HEADING", 149.9);
    // TODO: extract message

    /////////////////////////////////////////////////////////

  



    /////////////////////////////////////////////////////////

  }
  void ros_speed_listener(const std_msgs::msg::String &msg) {
    // TODO: extract message

    /////////////////////////////////////////////////////////

  



    /////////////////////////////////////////////////////////

    // publish
    Comms.Notify("NAV_SPEED", 149.9);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_latlon;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_depth;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_speed;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_heading;

  // TODO fix these publisher message types
  
  
};

bool OnConnect(void *pParam) {
  CMOOSCommClient *pC = reinterpret_cast<CMOOSCommClient *>(pParam);
  // from MOOS-IVP to ros base
  pC->Register("DESIRED_SPEED", 0.0);
  pC->Register("DESIRED_HEADING", 0.0);
  pC->Register("DESIRED_DEPTH", 0.0);
  return 0;
}

// Receives the 
bool OnMail(void *pParam) {
  CMOOSCommClient *pC = reinterpret_cast<CMOOSCommClient *>(pParam);
  MOOSMSG_LIST M;
  pC->Fetch(M);
  MOOSMSG_LIST::iterator q;
  for (q = M.begin(); q != M.end(); q++) {
    CMOOSMsg &msg = *q;
    std::string key = msg.GetKey();
    double value = msg.GetValue();
    std::cout << "\n";

    if (key == "DESIRED_SPEED") {
      auto message = frost_interfaces::msg::DesiredSpeed();
      std::cout << "=====PRINTING DESIRED_SPEED=====" << std::endl;
      message.desired_speed = value;
      desired_speed_publisher->publish(message);
    } else if (key == "DESIRED_HEADING") {
      auto message = frost_interfaces::msg::DesiredHeading();
      std::cout << "=====PRINTING DESIRED_HEADING=====" << std::endl;
      message.desired_heading = value;
      desired_heading_publisher_->publish(message);
    } else if (key == "DESIRED_DEPTH") {
      auto message = frost_interfaces::msg::DesiredDepth();
      std::cout << "=====PRINTING DESIRED_DEPTH=====" << std::endl;
      message.desired_depth = value;
      desired_depth_publisher_->publish(message);
    }

    q->Trace();
    std::cout << "\n";


  }
  return true;
}

int main(int argc, char *argv[]) {

  // MOOS stuff
  Comms.SetOnMailCallBack(OnMail, &Comms);
  Comms.SetOnConnectCallBack(OnConnect, &Comms);
  Comms.Run("localhost", 9000, "my_connection");

  // ROS 2 stuff
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MOOSBridge>());
  rclcpp::shutdown();

  return 0;
}