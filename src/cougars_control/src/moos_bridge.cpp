#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <sstream>


// ros2 stuff
#include "frost_interfaces/msg/desired_depth.hpp"
#include "frost_interfaces/msg/desired_heading.hpp"
#include "frost_interfaces/msg/desired_speed.hpp"
#include "frost_interfaces/msg/vehicle_status.hpp"
#include "seatrac_interfaces/msg/modem_status.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


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
using namespace std::chrono_literals;


// status variables


struct status {
    // state estimate of x,y
    double x_ = 0.0;
    double y_ = 0.0;
    // depth sensor
    double depth_ = 0.0;
    // modem heading
    double heading_ = 0.0;
    // error code (0 is no error)
    // non zero will signify error
    // TODO: define what kind of errors, if we want to define any of that
    int error_ = 0;
    // amount of waypoints hit (if you are not on a waypoint behavior, then this will read -1)
    int waypoint_number_ = 0;

    std::string vname_ = "";

    std::string curr_behavior_ = "";

    int dist_ = 0;

    int eta_ = 0;

    bool completed_ = false;
};
 
typedef struct status Status;

static Status vehicle_status;



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
rclcpp::Publisher<frost_interfaces::msg::VehicleStatus>::SharedPtr vehicle_status_publisher_;

// MOOS functions

class MOOSBridge : public rclcpp::Node {
public:
  MOOSBridge() : Node("moos_bridge") {

   

    this->declare_parameter("gps", "false");
    this->declare_parameter("sim", "false");


    // subscriptions -- depending on if in sim or in real life

    if (this->get_parameter("gps").as_string() == "true" && this->get_parameter("sim").as_string() == "true"){
      subscription_smoothed_output_ =
          this->create_subscription<nav_msgs::msg::Odometry>(
              "gps_odom", 10,
              std::bind(&MOOSBridge::ros_smoothed_output_listener, this, _1));
    }
    else{
      // vehicle status listener from the factor graph filter
      subscription_smoothed_output_ =
      this->create_subscription<nav_msgs::msg::Odometry>(
          "smoothed_output", 10,
          std::bind(&MOOSBridge::ros_smoothed_output_listener, this, _1));
      }
    // just grab the heading straight from the modem for now, there is a converter in cougars_sim
    actual_heading_subscription_ =
        this->create_subscription<seatrac_interfaces::msg::ModemStatus>(
            "modem_status", 10,
            std::bind(&MOOSBridge::actual_heading_callback, this, _1));

    actual_depth_subscription_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "depth_data", 10,
        std::bind(&MOOSBridge::actual_depth_callback, this, _1));


  
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
    vehicle_status_publisher_ = this->create_publisher<frost_interfaces::msg::VehicleStatus>(
            "vehicle_status", 10);

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MOOSBridge::status_update_callback, this));
  }

private:


  void actual_depth_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped &depth_msg) {
    vehicle_status.depth_ = -depth_msg.pose.pose.position.z;
    //Negate the z value in ENU to get postive depth value
  }

  void PrintStatusUpdate(){

    std::cout << "========================================\n";
    std::cout << "Vehicle Status Published to ROS2 network:\n\n";
    std::cout << "Waypoints Reached (WPT_IDX): " << vehicle_status.waypoint_number_ << "\n\n";
    std::cout << "Error update: " << vehicle_status.error_ << "\n";
    std::cout << "Mission Complete: " << vehicle_status.completed_ << "\n";
    std::cout << "Current Behavior (or last if completed): " << vehicle_status.curr_behavior_ << "\n";
    std::cout << "X est: " << vehicle_status.x_ << "\n";
    std::cout << "Y est: " << vehicle_status.y_ << "\n";
    std::cout << "Depth: " << vehicle_status.depth_ << "\n";
    std::cout << "Heading: " << vehicle_status.heading_ << "\n";
    std::cout << "Distance to next WPT: " << vehicle_status.dist_ << "\n";
    std::cout << "ETA: " << vehicle_status.eta_ << "\n";
    std::cout << "Name: " << vehicle_status.vname_ << "\n";

    std::cout << "========================================\n\n";


  }





  void status_update_callback()
      {
        auto message = frost_interfaces::msg::VehicleStatus();
        message.x = vehicle_status.x_;
        message.y = vehicle_status.y_;
        message.depth = vehicle_status.depth_;
        message.heading = vehicle_status.heading_;
        message.error = vehicle_status.error_;
        vehicle_status_publisher_->publish(message);
        PrintStatusUpdate();

        
  }
  // needs to listen to (x,y), depth, speed,
  // heading -->  NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING, NAV_DEPTH
  void
  ros_smoothed_output_listener(const nav_msgs::msg::Odometry &msg) {

    double nav_x, nav_y, nav_depth, nav_speed;


    if(this->get_parameter("sim").as_string() == "true"){

      nav_x = -msg.pose.pose.position.y;
      nav_y = msg.pose.pose.position.x;

    }
    else{

      nav_x = msg.pose.pose.position.x;
      nav_y = msg.pose.pose.position.y;

    }
    vehicle_status.x_ = nav_x;
    vehicle_status.y_ = nav_y;
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

    // heading_ = nav_heading;

    Comms.Notify("NAV_HEADING", nav_heading);
  }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_smoothed_output_;
  rclcpp::Subscription<seatrac_interfaces::msg::ModemStatus>::SharedPtr actual_heading_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      actual_depth_subscription_;
  
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
  pC->Register("IVPHELM_SUMMARY", 0.0);
  pC->Register("WPT_STAT",0.0);
  //NOTE: WPT_INDEX seems to not be working, so I hand to parse WPT_STAT string
  // pC->Register("WPT_INDEX",0.0);
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


void ParseWaypointNumber(std::string waypoint_status){

    std::string str1 = "index=";
    std::string str2 = ",hits";
    int idx1 =  waypoint_status.find(str1);
    int idx2 = waypoint_status.find(str2);

    std::string num_waypoints = waypoint_status.substr(idx1 + str1.length(), idx2 - (str1.length() + idx1));

    std::stringstream ss;
    ss << num_waypoints;
    ss >> vehicle_status.waypoint_number_;

}

void ParseVName(std::string waypoint_status){

  std::string str1 = "vname=";
  std::string str2 = ",behavior";
  int idx1 =  waypoint_status.find(str1);
  int idx2 = waypoint_status.find(str2);

  std::string vname = waypoint_status.substr(idx1 + str1.length(), idx2 - (str1.length() + idx1));

  vehicle_status.vname_ = vname;

  
}
void ParseWaypointCurrentBehavior(std::string waypoint_status){

  std::string str1 = "behavior-name=";
  std::string str2 = ",index";
  int idx1 =  waypoint_status.find(str1);
  int idx2 = waypoint_status.find(str2);

  std::string current_behavior = waypoint_status.substr(idx1 + str1.length(), idx2 - (str1.length() + idx1));

  vehicle_status.curr_behavior_ = current_behavior;

}
void ParseDistanceToNextWaypoint(std::string waypoint_status){

  std::string str1 = "dist=";
    std::string str2 = ",eta";
    int idx1 =  waypoint_status.find(str1);
    int idx2 = waypoint_status.find(str2);

    std::string dist = waypoint_status.substr(idx1 + str1.length(), idx2 - (str1.length() + idx1));

    std::stringstream ss;
    ss << dist;
    ss >> vehicle_status.dist_;
}

void ParseETA(std::string waypoint_status){

  std::string str1 = "eta=";
    // std::string str2 = "\0";
    int idx1 =  waypoint_status.find(str1);
    int idx2 = waypoint_status.length(); //waypoint_status.find(str2);

    std::string eta = waypoint_status.substr(idx1 + str1.length(), idx2 - (str1.length() + idx1));

    std::stringstream ss;
    ss << eta;
    ss >> vehicle_status.eta_;
}

void CompleteMission(bool completed){
  vehicle_status.completed_ = completed;
}


void ParseWaypointStatus(std::string waypoint_status){

      // status message is of something like:
      // vname=alpha,behavior-name=waypt_survey,index=2,hits=2/2,cycles=0,dist=42,eta=12
      // TODO: if we want, parse cycles and hits.

      if(waypoint_status.find("completed") == std::string::npos){
        ParseWaypointNumber(waypoint_status);
        ParseWaypointCurrentBehavior(waypoint_status);
        ParseDistanceToNextWaypoint(waypoint_status);
        ParseVName(waypoint_status);
        ParseETA(waypoint_status);
        CompleteMission(false);
      }
      else{
        CompleteMission(true);
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
    // std::cout << key << std::endl;
    if (key == "WPT_STAT"){
      std::string status = msg.GetString();
      // std::cout<<status<<std::endl;
      ParseWaypointStatus(status);
    }
    else{

      double value = msg.GetDouble();
      PublishDesiredValue(value, key);
      // std::cout << key << ": " << value << std::endl;

    }

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