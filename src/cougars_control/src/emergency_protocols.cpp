#include <memory>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <numeric>
#include <functional>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "frost_interfaces/msg/battery_status.hpp"
#include "frost_interfaces/msg/leak_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "frost_interfaces/msg/u_command.hpp"
#include "std_msgs/msg/int32.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile);


static double depth_val;

class EmergencyProtocols : public rclcpp::Node
{


public:
  //////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////// CONSTRUCTOR ///////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////
  EmergencyProtocols()
  : Node("emergency_protocols")
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////// PARAMETERS /////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////

    // set all parmeters in yaml file

    // controlling what you want to monitor:
    // if these flags are set to true, then emergency
    // handling will occur in any relevant subscribers
  
    this->declare_parameter("monitor_leak", true);
    this->declare_parameter("monitor_low_battery", true);
    this->declare_parameter("monitor_moos", true);
    this->declare_parameter("monitor_factor_graph", true);
    this->declare_parameter("monitor_depth", true);
    this->declare_parameter("monitor_controls", true);
    this->declare_parameter("monitor_heartbeat", true);
    this->declare_parameter("monitor_waypoint_progress", true);
    this->declare_parameter("monitor_mission_timout", true);
    this->declare_parameter("monitor_dive_ability", true);
    this->declare_parameter("monitor_collision", true);


    // safety parameters (voltage, current, depth, etc.)
    this->declare_parameter("deepest_safe_depth", -0.7); // meters
    this->declare_parameter("critical_voltage", 14.0); // volts
    //  TODO: add more safety parameters as needed



    //////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////// SUBSCRIPTIONS and Timers ///////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////


    depth_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("depth_data", 
            qos, std::bind(&EmergencyProtocols::depth_callback, this, _1));
    leak_subscription_ = this->create_subscription<frost_interfaces::msg::LeakStatus>("leak/data", 
            qos, std::bind(&EmergencyProtocols::leak_callback, this, _1));
    battery_subscription_ = this->create_subscription<frost_interfaces::msg::BatteryStatus>("battery/data", 
            qos, std::bind(&EmergencyProtocols::battery_callback, this, _1));
    // smoothed_output_subscription_ =
    //     this->create_subscription<nav_msgs::msg::Odometry>("smoothed_output", 
    //         qos,std::bind(&EmergencyProtocols::factor_graph_callback, this, _1));
    // vehicle_status_moos_subscription_ = this->create_subscription<frost_interfaces::msg::VehicleStatus>("vehicle_status", 
    //         qos, std::bind(&EmergencyProtocols::moos_status_callback, this, _1));


    command_subscription_ =
    this->create_subscription<frost_interfaces::msg::UCommand>(
        "controls/command", 10,
        std::bind(&EmergencyProtocols::command_callback, this, _1));


    status_publisher_ = this->create_publisher<std_msgs::msg::Int32>("safety_status", 10);

    this->okay = true;
    this->init = false;

    depth_val = 1.0;




    // status publisher
    // TODO: Add safety status publisher,


    // timers
    monitor_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5000),
        std::bind(&EmergencyProtocols::monitor_timer_callback, this));
    handle_factor_graph_ = this->create_wall_timer(
        std::chrono::milliseconds(5000),
        std::bind(&EmergencyProtocols::handle_factor_graph_callback, this));


  
    
    /// init variables
    counter = 0;

    // vector to store important topics ahd whether they have publishers or not
    // topics_publishing.insert({{"smoothed_output", true}, 
    //                           {"battery/data", true},
    //                           {"leak/data", true}, 
    //                           {"depth/data", true},
    //                           {"modem_imu", true},
    //                           {"desired_depth", true},
    //                           {"desired_heading", true},
    //                           {"desired_speed", true},
    //                           {"vehicle_status", true},
    //                           {"kinematics/command", true},
    //                           {"controls/command", true},
    //                           {"gps_odom", true},
    //                           {"extended_fix", true},
    //                           {"dvl_dead_reckoning", true}});


    topics_publishing.insert({{"battery/data", true},
                              {"leak/data", true}, 
                              {"depth_data", true},
                                {"modem_status", true},
                                {"dvl/dead_reckoning", true}});
  

    //////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////// INIT EMERGENCY REQUESTS ///////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////

    
    ////////////////////////////
    // (dis)arm thruster request
    ////////////////////////////
    // in response to:
    //      * leak 
    //      * low battery
    //      * TODO: add more as needed
    arm_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    arm_thruster_client = this->create_client<std_srvs::srv::SetBool>("arm_thruster");
    while (!arm_thruster_client->wait_for_service(1s)){
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        break;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    ///////////////////////////////////
    // override fins to surface request
    ///////////////////////////////////
    // in response to:
    //      * too deep  
    //      * TODO: add more as needed
    surface_fin_override_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    surface_fin_override_client = this->create_client<std_srvs::srv::SetBool>("surface");
    while (!surface_fin_override_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        break;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }


    ///////////////////////////////////
    // TODO: add other requests as necessary
    ///////////////////////////////////
    // in response to:
    //      * TODO: add more 
    //      * ...



  } // end constructor

  

 private:




void surface(){
  if (this->init){
    RCLCPP_INFO(this->get_logger(), "Surface");
    // this->okay = false;
    // surface_fin_override_request->data = true; // true to override
    // send_set_bool_req(surface_fin_override_request, surface_fin_override_client);
  } 
}

void disarm(){
  if (this->init){
    RCLCPP_INFO(this->get_logger(), "Disarm");
    // this->okay = false;
    // arm_request->data = false; // make false to diarm the thruster
    // send_set_bool_req(arm_request, arm_thruster_client);
  }
}
  

//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////  CB's ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////


// monitor timer helpers


bool update_publishers(){

  bool all_topics_have_publishers = true;

  // // look at all topics of interest
  // for (auto itr0 : topics_publishing){

  //   bool topic_has_publishers = false;

  //   // see if everything in topics_publishing is indeed being published to
  //   for(auto itr : get_topic_names_and_types()){
  //     if(itr.first.find(itr0.first) != std::string::npos){
  //       if (count_publishers(itr.first) > 0){
  //         topic_has_publishers = true;
  //       }
  //     }
  //   }
  //   topics_publishing[itr0.first.c_str()] = topic_has_publishers;
  //   if (!topic_has_publishers){
  //     all_topics_have_publishers = false;
  //     this->okay = false;
  //     RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "%s has no publishers!", itr0.first.c_str());
  //   }
  // }

  return all_topics_have_publishers;

}


////////////////////////////////////
//  Monitoring Publishers 
//  to important topics
///////////////////////////////////

  // this timer looks through all topics on the network and checks for crucial topics such as 
  // smoothed_output, leak/data, etc. Key topics are in topics_publishing map
  void monitor_timer_callback() {

    if (counter < 1){
      counter++;
    }
    else{
      if(!update_publishers()){

        this->okay = false;
        // handle topics 
        if (!topics_publishing["modem_status"]){
          surface();
          while(depth_val < -0.1);
          disarm();
        }
      }
    }


    auto message = std_msgs::msg::Int32();
    message.data = int32_t(this->okay);
    status_publisher_->publish(message);

  }



  void command_callback(const frost_interfaces::msg::UCommand &msg) {
    this->init = true;
  }



///////////////////////////////////
//  Factor Graph Emergency
///////////////////////////////////
//  Requests to be sent:
//      * surface_fin_override
//      * return_to_home 


  void handle_factor_graph_callback(){
    if (this->get_parameter("monitor_factor_graph").as_bool()){
      // handle factor graph emergencies here
      // TODO: discuss how we handle this
      // surface
      // disarm thruster
      // restart factor graph

      // if(!topics_publishing["smoothed_output"]){

      // }
    }
  }



///////////////////////////////////
//  Depth Emergency
///////////////////////////////////
//  Requests to be sent:
//      * surface_fin_override 

void depth_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg){
  if (this->get_parameter("monitor_depth").as_bool()){
    depth_val = msg.pose.pose.position.z;

    // RCLCPP_INFO(this->get_logger(), "%f\n" , depth_val);

    // if the current depth is too deep, surface
    if (depth_val < this->get_parameter("deepest_safe_depth").as_double()){
        surface();
    }

    // TODO: put any other cases we may deduce from msg
    // ... 
    // ...
  }
}



///////////////////////////////////
// Battery Emergency
///////////////////////////////////
//  Requests to be sent:
//      * disarm thruster 

void battery_callback(const frost_interfaces::msg::BatteryStatus &msg){

  if (this->get_parameter("monitor_low_battery").as_bool()){
    double curr_volt = msg.voltage;
    if (curr_volt < this->get_parameter("critical_voltage").as_double()){
      disarm();
    }

    // TODO: put any other cases we may deduce from msg
    // ... 
    // ...
  }
}



///////////////////////////////////
// Leak Emergency
///////////////////////////////////
//  Requests to be sent:
//      * disarm thruster 

void leak_callback(const frost_interfaces::msg::LeakStatus &msg){

  if (this->get_parameter("monitor_leak").as_bool()){
    if (msg.leak){
      disarm();
      RCLCPP_INFO(this->get_logger(), "Emergency request being sent in response to leak");
      
    }

    // TODO: put any other cases we may deduce from msg
    // ... 
    // ...

  }
}

///////////////////////////////////
// MOOS Emergency
///////////////////////////////////
//
// THIS IS WHAT IS PUBLISHED FROM MOOS BRIDGE:
//
//         message.x = vehicle_status.x_;
//         message.y = vehicle_status.y_;
//         message.depth = vehicle_status.depth_;
//         message.heading = vehicle_status.heading_;
//         message.error = vehicle_status.error_;
//         vehicle_status_publisher_->publish(message);
//
//      # VehicleStatus message:

        // # header
        // std_msgs/Header header

        // # status members
        // float64 x
        // float64 y
        // float64 depth
        // float64 heading
        // int64 error
        // int8 behavior_number
        // int8 waypoints_reached
        // bool mission_complete
        // float64 dist_to_next_wpt
//         
//    if error is non zero, BHV_ERROR:
//        * surface, return to home
// 
//    if dist_to_next_wpt is increasing for 
//    a long time and never getting closer
//        * surfac, return to home

// void moos_status_callback(const frost_interfaces::msg::VehicleStatus &msg){
//   if(this->get_parameter("monitor_moos").as_bool()){
//     // TODO: handle MOOS problems here


//   }
// }


//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// SEND REQUEST HELPER //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

// works just for SetBool requests
 void send_set_bool_req(std::shared_ptr<std_srvs::srv::SetBool::Request> request, 
                        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client){
    if (client->service_is_ready()){
      // put in a callback function
      auto  result = client->async_send_request(request,
      [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            try {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Service response: %s", response->message.c_str());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        }
    );
    }
    else{
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "service not ready");
    }

  }

 


//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// REQUESTS & CLIENTS ///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

  // requests  TODO: put more requests here

  std::shared_ptr<std_srvs::srv::SetBool::Request>  arm_request;
  std::shared_ptr<std_srvs::srv::SetBool::Request>  surface_fin_override_request;


  // emergency service slients, TODO: add more service clients
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_thruster_client; 
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr surface_fin_override_client;



//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// SUBSCRIPTION/TIMER DECLARATIONS //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_subscription_;
  rclcpp::Subscription<frost_interfaces::msg::LeakStatus>::SharedPtr leak_subscription_;
  rclcpp::Subscription<frost_interfaces::msg::BatteryStatus>::SharedPtr battery_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr smoothed_output_subscription_;
//   rclcpp::Subscription<frost_interfaces::msg::VehicleStatus>::SharedPtr vehicle_status_moos_subscription_;
  rclcpp::Subscription<frost_interfaces::msg::UCommand>::SharedPtr command_subscription_;



  //publisher
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_publisher_;

  // timers 
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::TimerBase::SharedPtr handle_factor_graph_;
  int counter;

  bool okay;
  bool init;

   // topic name vector
  std::map<std::string, bool> topics_publishing;
  
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyProtocols>());
  rclcpp::shutdown();

  return 0;
}
