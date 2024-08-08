#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "frost_interfaces/msg/u_command.hpp"
#include <frost_interfaces/msg/control_config.hpp>
#include <frost_interfaces/msg/desired_depth.hpp>
#include <frost_interfaces/msg/desired_heading.hpp>
#include <frost_interfaces/msg/desired_speed.hpp>
#include "pid_control.h"

using namespace std::chrono_literals;

// ros config values
#define TIMER_PID_PERIOD 10ms // 100 Hz

// default actuator positions
#define DEFAULT_SERVO 90
#define THRUSTER_OFF 1500

class DemoControl : public rclcpp::Node
{
  public:
    DemoControl()
    : Node("demo_control")
    {
      u_command_publisher_ = this->create_publisher<frost_interfaces::msg::UCommand>("control_command", 10);

      control_config_subscription_ = this->create_subscription<frost_interfaces::msg::ControlConfig>(
      "control_config", 10, std::bind(&DemoControl::config_callback, this, _1));

      desired_depth_subscription_ = this->create_subscription<frost_interfaces::msg::DesiredDepth>(
      "desired_depth", 10, std::bind(&DemoControl::depth_callback, this, _1));

      desired_heading_subscription_ = this->create_subscription<frost_interfaces::msg::DesiredHeading>(
      "desired_heading", 10, std::bind(&DemoControl::heading_callback, this, _1));

      desired_speed_subscription_ = this->create_subscription<frost_interfaces::msg::DesiredSpeed>(
      "desired_speed", 10, std::bind(&DemoControl::speed_callback, this, _1));

      // TODO: add depth and DVL subscriptions

      pid_timer_ = this->create_wall_timer(
      TIMER_PID_PERIOD, std::bind(&DemoControl::timer_callback, this));
    }

  private:
    void config_callback(const frost_interfaces::msg::ControlConfig & config_msg) const
    {
      
      // calibrate the depth PID controller
      myDepthPID.calibrate(config_msg.depth_kp, config_msg.depth_ki,
                          config_msg.depth_kd, config_msg.depth_min_output,
                          config_msg.depth_max_output, TIMER_PID_PERIOD,
                          config_msg.depth_bias);
      // calibrate the heading PID controller
      myHeadingPID.calibrate(config_msg.heading_kp, config_msg.heading_ki,
                            config_msg.heading_kd, config_msg.heading_min_output,
                            config_msg.heading_max_output, TIMER_PID_PERIOD,
                            config_msg.heading_bias);

      // calibrate the velocity PID controller
      myVelocityPID.calibrate(
          config_msg.velocity_kp, config_msg.velocity_ki, config_msg.velocity_kd,
          config_msg.velocity_min_output, config_msg.velocity_max_output,
          TIMER_PID_PERIOD, config_msg.velocity_bias);

      configured = true;
    }

    void depth_callback(const frost_interfaces::msg::DesiredDepth & depth_msg) const
    {
      desired_depth_msg = depth_msg;
    }

    void heading_callback(const frost_interfaces::msg::DesiredHeading & heading_msg) const
    {
      desired_heading_msg = heading_msg;
    }

    void speed_callback(const frost_interfaces::msg::DesiredSpeed & speed_msg) const
    {
      desired_speed_msg = speed_msg;
    }

    // TODO: add depth and DVL callbacks (update class sensor variables)

    void timer_callback()
    {
      auto message = frost_interfaces::msg::UCommand();

      if (configured) {

        //////////////////////////////////////////////////////////
        // LOW-LEVEL CONTROLLER CODE STARTS HERE
        // - Reference wanted values using the desired_depth_msg,
        //   desired_heading_msg, and desired_speed_msg objects
        //////////////////////////////////////////////////////////

        // TODO: reset the dead reckoning on the dvl as soon as we start moving (?)

        depth_pos = myDepthPID.compute(desired_depth_msg->desired_depth, depth);
        heading_pos =
            myHeadingPID.compute(desired_heading_msg->desired_heading, yaw);
        velocity_level =
            myVelocityPID.compute(desired_speed_msg->desired_speed, x_velocity);

        // TODO: update message
        message.data = "Hello, world! " + std::to_string(count_++);
      } else {

        // TODO: update message with default values (DEFAULT_SERVO, THRUSTER_OFF)
        message.data = "Hello, world! " + std::to_string(count_++);
      }

      u_command_publisher_->publish(message);

      //////////////////////////////////////////////////////////
      // LOW-LEVEL CONTROLLER CODE ENDS HERE
      //////////////////////////////////////////////////////////
    }

    // flags on start
    bool configured = false;

    // micro-ROS objects
    rclcpp::TimerBase::SharedPtr pid_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr u_command_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_config_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desired_depth_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desired_heading_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desired_speed_subscription_;
    // TODO: add depth and DVL subscriptions

    // micro-ROS messages
    frost_interfaces::msg::DesiredDepth desired_depth_msg;
    frost_interfaces::msg::DesiredHeading desired_heading_msg;
    frost_interfaces::msg::DesiredSpeed desired_speed_msg;

    // control objects
    PID_Control myHeadingPID;
    PID_Control myDepthPID;
    PID_Control myVelocityPID;

    // class sensor variables
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
    float x_velocity = 0.0;
    float pressure = 0.0;
    float depth = 0.0;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoControl>());
  rclcpp::shutdown();
  return 0;
}