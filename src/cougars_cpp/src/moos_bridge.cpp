#include <functional>
#include <memory>

// ros2 stuff
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "MOOS/libMOOS/Comms/MOOSAsyncCommClient.h"
#include "MOOS/libMOOS/Comms/MOOSCommClient.h"
#include "MOOS/libMOOS/Comms/MOOSMsg.h"
#include "MOOS/libMOOS/Utils/CommandLineParser.h"
#include <exception>

using std::placeholders::_1;

MOOS::MOOSAsyncCommClient Comms;
typedef std::vector<CMOOSMsg> MsgVector;
typedef std::vector<MOOS::ClientCommsStatus> CommsStatusVector;

// MOOS functions

class MOOSBridge : public rclcpp::Node {
public:
  MOOSBridge() : Node("moos_bridge") {

    init_i();

    // CMOOSCommClient::Register("NAV_X", 0.0);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&Bridge::ros_listener, this, _1));
  }

  void init_i() { i = 1; }
  void increment_i() { i++; }
  int get_i() { return i; }

private:
  int i;

  void ros_listener(const std_msgs::msg::String &msg) {
    increment_i();
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

    // translate message

    // notify

    if (get_i() % 10 == 0) {
      Comms.Notify("NAV_X", 149.9);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

bool OnConnect(void *pParam) {
  CMOOSCommClient *pC = reinterpret_cast<CMOOSCommClient *>(pParam);

  pC->Register("X", 0.0);

  // from MOOS-IVP to ros base
  pC->Register("DESIRED_SPEED", 0.0);
  pC->Register("DESIRED_HEADING", 0.0);
  pC->Register("DESIRED_DEPTH", 0.0);
  return 0;
}

bool OnMail(void *pParam) {

  CMOOSCommClient *pC = reinterpret_cast<CMOOSCommClient *>(pParam);
  MOOSMSG_LIST M;
  pC->Fetch(M);
  MOOSMSG_LIST::iterator q;
  for (q = M.begin(); q != M.end(); q++) {
    CMOOSMsg &msg = *q;
    std::string key = msg.GetKey();
    std::cout << "\n";

    if (key == "DESIRED_SPEED") {
      std::cout << "=====PRINTING DESIRED_SPEED=====" << std::endl;

    } else if (key == "DESIRED_HEADING") {
      std::cout << "=====PRINTING DESIRED_HEADING=====" << std::endl;

    } else if (key == "DESIRED_DEPTH") {
      std::cout << "=====PRINTING DESIRED_DEPTH=====" << std::endl;
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