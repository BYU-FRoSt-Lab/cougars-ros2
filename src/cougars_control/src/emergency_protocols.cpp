#include <memory>

#include "rclcpp/rclcpp.hpp"

class EmergencyParameterHandler : public rclcpp::Node
{
public:
  EmergencyParameterHandler()
  : Node("emergency_parameter_handler")
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

    // Create a parameter subscriber that can be used to monitor parameter changes
    // (for this node's parameters as well as other nodes' parameters)
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto handle_problem_callback = [this](const rclcpp::Parameter & p) {

        // display emergency 

        if (p.as_bool() == true){

          std::string parameter = p.get_name().c_str();

          RCLCPP_INFO(
            this->get_logger(), "\nPROBLEM DETECTED: Parameter \"%s\" = \"%s\"",
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
            // TODO: Implement service to deal with emergency
          }
          else if (parameter == "low_battery_detected"){
            // TODO: Implement service to deal with emergency
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

 private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
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
 

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyParameterHandler>());
  rclcpp::shutdown();

  return 0;
}