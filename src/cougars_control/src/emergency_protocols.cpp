#include <memory>
#include <chrono>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"



using namespace std::chrono_literals;


class EmergencyProtocols : public rclcpp::Node
{

public:

  // constructor 
  EmergencyProtocols()
  : Node("emergency_protocols")
  {
     // problems that could be detected
    this->declare_parameter("leak_detected", false);
    this->declare_parameter("low_battery_detected", false);
    this->declare_parameter("collision_detected", false);
    this->declare_parameter("moos_error_detected", false);
    this->declare_parameter("factor_graph_error_detected", false);
    this->declare_parameter("controls_node_error_detected", false);
    this->declare_parameter("no_heartbeat_error_detected", false);
    this->declare_parameter("no_progress_waypoint_error_detected", false);
    this->declare_parameter("too_deep_detected", false);
    this->declare_parameter("mission_timeout_detected", false);
    this->declare_parameter("unable_to_dive_detected", false);


    // emergency requests TODO: Put more requests
    disarm_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    disarm_request->data = false; // to disarm thruster
    while (!disarm_thruster_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        break;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }


    // Create a parameter subscriber that can be used to monitor parameter changes
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    // handles each emergency case
    auto handle_problem_callback = [this](const rclcpp::Parameter & p) {

      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "ALERT");


        std::cout << "in problem handler\n";
        // if the emergency flag is raised to true, then carry out the service
        if (p.as_bool() == true){

          std::string parameter = p.get_name().c_str();

          RCLCPP_WARN(
            this->get_logger(), "\nPROBLEM DETECTED: Parameter \"%s\" = %s",
            p.get_name().c_str(),
            p.as_bool() ? "true" : "false");


          if (parameter ==  "collision_detected"){
            // TODO: Implement service to deal with emergency
          }
          else if (parameter == "controls_node_error_detected"){
            // TODO: Implement service to deal with emergency
          }
          else if (parameter == "factor_graph_error_detected"){
            // TODO: Implement service to deal with emergency
          }
          else if (parameter == "leak_detected"){
            // Thruster disarm service client and request
            send_disarm_req(disarm_request);
            
          }
          else if (parameter == "low_battery_detected"){
            send_disarm_req(disarm_request);
          }
          else if (parameter == "mission_timeout_detected"){
            // TODO: Implement service to deal with emergency
          }
          else if (parameter == "moos_error_detected"){
            // TODO: Implement service to deal with emergency
          }
          else if (parameter == "no_heartbeat_error_detected"){
            // TODO: Implement service to deal with emergency
          }
          else if (parameter == "no_progress_waypoint_error_detected"){
            // TODO: Implement service to deal with emergency
          }
          else if (parameter == "too_deep_detected"){
            // TODO: Implement service to deal with emergency
          }
          else if (parameter == "unable_to_dive_detected"){
            // TODO: Implement service to deal with emergency
          }
        
        } 
      };


    // collision_detected
    // controls_node_error_detected
    // factor_graph_error_detected
    // leak_detected
    // low_battery_detected
    // mission_timeout_detected
    // moos_error_detected
    // no_heartbeat_error_detected
    // no_progress_waypoint_error_detected
    // too_deep_detected
    // unable_to_dive_detected
    collision_detected_handle_ = param_subscriber_->add_parameter_callback("collision_detected", handle_problem_callback);
    controls_node_error_detected_handle_ = param_subscriber_->add_parameter_callback("controls_node_error_detected", handle_problem_callback);
    factor_graph_error_detected_handle_ = param_subscriber_->add_parameter_callback("factor_graph_error_detected", handle_problem_callback);
    leak_detected_handle_ = param_subscriber_->add_parameter_callback("leak_detected", handle_problem_callback);
    low_battery_detected_handle_ = param_subscriber_->add_parameter_callback("low_battery_detected", handle_problem_callback);
    mission_timeout_detected_handle_ = param_subscriber_->add_parameter_callback("mission_timeout_detected", handle_problem_callback);
    moos_error_detected_handle_ = param_subscriber_->add_parameter_callback("moos_error_detected", handle_problem_callback);
    no_heartbeat_error_detected_handle = param_subscriber_->add_parameter_callback("no_heartbeat_error_detected", handle_problem_callback);
    no_progress_waypoint_error_detected_handle_ = param_subscriber_->add_parameter_callback("no_progress_waypoint_error_detected", handle_problem_callback);
    too_deep_detected_handle_ = param_subscriber_->add_parameter_callback("too_deep_detected", handle_problem_callback);
    unable_to_dive_detected_handle_ = param_subscriber_->add_parameter_callback("unable_to_dive_detected", handle_problem_callback);

  }

  
  // request functions

  // thruster disarm request send 
  void send_disarm_req(std::shared_ptr<std_srvs::srv::SetBool::Request> request){
    if (disarm_thruster_client->service_is_ready()){

      // put in a callback function
      auto  result = disarm_thruster_client->async_send_request(request,
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

 private:


  // requests  TODO: put more requests here

  std::shared_ptr<std_srvs::srv::SetBool::Request>  disarm_request;

  // param subsciber
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;

  // callback handles
  std::shared_ptr<rclcpp::ParameterCallbackHandle> collision_detected_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> controls_node_error_detected_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> leak_detected_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> factor_graph_error_detected_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> low_battery_detected_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> mission_timeout_detected_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> moos_error_detected_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> no_heartbeat_error_detected_handle;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> no_progress_waypoint_error_detected_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> too_deep_detected_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> unable_to_dive_detected_handle_;


  // emergency service slients, TODO: add more service clients
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr disarm_thruster_client = this->create_client<std_srvs::srv::SetBool>("arm_thruster");


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyProtocols>());
  rclcpp::shutdown();

  return 0;
}