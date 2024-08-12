#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <frost_interfaces/msg/desired_depth.hpp>
#include <frost_interfaces/msg/desired_heading.hpp>
#include <frost_interfaces/msg/desired_speed.hpp>
#include <frost_interfaces/msg/u_command.hpp>
// TODO: include depth and DVL messages
#include "pid_control.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

// ros config values
#define TIMER_PID_PERIOD '10ms' // 100 Hz

// default actuator positions
#define DEFAULT_SERVO 0
#define THRUSTER_OFF 0

class FinControl : public rclcpp::Node {
public:
  FinControl() : Node("fin_control") {
    u_command_publisher_ =
        this->create_publisher<frost_interfaces::msg::UCommand>(
            "control_command", 10);

    desired_depth_subscription_ =
        this->create_subscription<frost_interfaces::msg::DesiredDepth>(
            "desired_depth", 10,
            std::bind(&FinControl::depth_callback, this, _1));

    desired_heading_subscription_ =
        this->create_subscription<frost_interfaces::msg::DesiredHeading>(
            "desired_heading", 10,
            std::bind(&FinControl::heading_callback, this, _1));

    desired_speed_subscription_ =
        this->create_subscription<frost_interfaces::msg::DesiredSpeed>(
            "desired_speed", 10,
            std::bind(&FinControl::speed_callback, this, _1));

    // TODO: fix message types for depth and DVL
    depth_subscription_ =
        this->create_subscription<frost_interfaces::msg::Depth>(
            "depth", 10, std::bind(&FinControl::depth_callback, this, _1));

    dvl_subscription_ = this->create_subscription<frost_interfaces::msg::DVL>(
        "dvl", 10, std::bind(&FinControl::dvl_callback, this, _1));

    pid_timer_ = this->create_wall_timer(
        TIMER_PID_PERIOD, std::bind(&FinControl::timer_callback, this));
  }

private:
  void
  depth_callback(const frost_interfaces::msg::DesiredDepth &depth_msg) const {
    desired_depth_msg = depth_msg;
  }

  void heading_callback(
      const frost_interfaces::msg::DesiredHeading &heading_msg) const {
    desired_heading_msg = heading_msg;
  }

  void
  speed_callback(const frost_interfaces::msg::DesiredSpeed &speed_msg) const {
    desired_speed_msg = speed_msg;
  }

  // TODO: update message types for depth and DVL
  void depth_callback(const frost_interfaces::msg::Depth &depth_msg) const {
    depth = depth_msg.depth;
  }

  void dvl_callback(const frost_interfaces::msg::DVL &dvl_msg) const {
    yaw = dvl_msg.yaw;
    x_velocity = dvl_msg.x_velocity;
  }

  void timer_callback() {
    auto message = frost_interfaces::msg::UCommand();

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

    message.fin[0] = depth_pos;
    message.fin[1] = depth_pos; // TODO: counter-rotation offset?
    message.fin[2] = heading_pos;
    message.thruster =
        velocity_level; // TODO: some conversion here on Teensy side

    u_command_publisher_->publish(message);

    //////////////////////////////////////////////////////////
    // LOW-LEVEL CONTROLLER CODE ENDS HERE
    //////////////////////////////////////////////////////////
  }

  // micro-ROS objects
  rclcpp::TimerBase::SharedPtr pid_timer_;
  rclcpp::Publisher<frost_interfaces::msg::UCommand>::SharedPtr
      u_command_publisher_;
  rclcpp::Subscription<frost_interfaces::msg::DesiredDepth>::SharedPtr
      desired_depth_subscription_;
  rclcpp::Subscription<frost_interfaces::msg::DesiredHeading>::SharedPtr
      desired_heading_subscription_;
  rclcpp::Subscription<frost_interfaces::msg::DesiredSpeed>::SharedPtr
      desired_speed_subscription_;
  // TODO: update message types for depth and DVL
  rclcpp::Subscription<frost_interfaces::msg::Depth>::SharedPtr
      depth_subscription_;
  rclcpp::Subscription<frost_interfaces::msg::DVL>::SharedPtr dvl_subscription_;

  // micro-ROS messages
  frost_interfaces::msg::DesiredDepth desired_depth_msg;
  frost_interfaces::msg::DesiredHeading desired_heading_msg;
  frost_interfaces::msg::DesiredSpeed desired_speed_msg;

  // control objects
  PID_Control myHeadingPID;
  PID_Control myDepthPID;
  PID_Control myVelocityPID;

  // class sensor variables
  float yaw = 0.0;
  float x_velocity = 0.0;
  float depth = 0.0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FinControl>());
  rclcpp::shutdown();
  return 0;
}