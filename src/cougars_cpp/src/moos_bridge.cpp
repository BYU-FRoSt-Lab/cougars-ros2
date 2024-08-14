#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>
#include <memory>

// ros2 stuff
#include "frost_interfaces/msg/desired_depth.hpp"
#include "frost_interfaces/msg/desired_heading.hpp"
#include "frost_interfaces/msg/desired_speed.hpp"
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

#define PI 3.14159265
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

    // ros listeners

    // TODO: change these to the correct topics and message types
    subscription_vehicle_status =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "vehicle_status", 10,
            std::bind(&MOOSBridge::ros_vehicle_status_listener, this, _1));

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
  // needs to listen to current latitude and longitude (x,y), depth, speed,
  // heading -->  NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING, NAV_DEPTH
  void ros_vehicle_status_listener(nav_msgs::msg::Odometry &msg) {

    double nav_x, nav_y, nav_depth, nav_heading, nav_speed;

    nav_x = msg.pose.pose.position.x;
    nav_y = msg.pose.pose.position.y;
    nav_depth = -1.0 * msg.pose.pose.position.z;
    nav_speed = msg.twist.twist.linear.x;

    // from quaternion, get heading
    Eigen::Quaterniond q;
    q.x() = msg.pose.pose.orientation.x;
    q.y() = msg.pose.pose.orientation.y;
    q.z() = msg.pose.pose.orientation.z;
    q.w() = msg.pose.pose.orientation.w;

    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    double yaw = euler[0];
    nav_heading = -1.0 * yaw * (180.0 / PI);

    Comms.Notify("NAV_X", nav_x);
    Comms.Notify("NAV_Y", nav_y);
    Comms.Notify("NAV_DEPTH", nav_depth);
    Comms.Notify("NAV_SPEED", nav_speed);
    Comms.Notify("NAV_HEADING", nav_heading);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      subscription_vehicle_status;
  // TODO fix these publisher message types
};

bool OnConnect(void *pParam) {
  CMOOSCommClient *pC = reinterpret_cast<CMOOSCommClient *>(pParam);
  // from MOOS-IVP to ros base
  pC->Register("DESIRED_SPEED", 0.0);
  pC->Register("DESIRED_HEADING", 0.0);
  pC->Register("DESIRED_DEPTH", 0.0);

  // std::string command = "uPokeDB " + MOOS_MISSION_DIR + "coug.moos" + " " +
  // variable + "=" + value " , MOOS_MANUAL_OVERIDE=false"; int result =
  // system(command.c_str());
  return 0;
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
    std::cout << "\n";

    if (key == "DESIRED_SPEED") {
      auto message = frost_interfaces::msg::DesiredSpeed();
      std::cout << "=====PRINTING DESIRED_SPEED=====" << std::endl;
      message.desired_speed = value;
      desired_speed_publisher_->publish(message);
    } else if (key == "DESIRED_HEADING") {
      auto message = frost_interfaces::msg::DesiredHeading();
      std::cout << "=====PRINTING DESIRED_HEADING=====" << std::endl;
      if (value < 0.0) {
        message.desired_heading = value + 360.0;
      } else {
        message.desired_heading = value;
      }

      desired_heading_publisher_->publish(message);
    } else if (key == "DESIRED_DEPTH") {
      auto message = frost_interfaces::msg::DesiredDepth();
      std::cout << "=====PRINTING DESIRED_DEPTH=====" << std::endl;
      message.desired_depth = value;
      desired_depth_publisher_->publish(message);
    }

    // q->Trace();
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