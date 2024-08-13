#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cougars_cpp/pid_control.h"
#include "dvl_msgs/msg/dvldr.hpp"
#include "frost_interfaces/msg/desired_depth.hpp"
#include "frost_interfaces/msg/desired_heading.hpp"
#include "frost_interfaces/msg/desired_speed.hpp"
#include "frost_interfaces/msg/u_command.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// TODO: might need to include pid_control.h in CMake file?

using namespace std::chrono_literals;
using std::placeholders::_1;

// ros config values
#define PID_TIMER_PERIOD_MS std::chrono::milliseconds(10)
#define PID_TIMER_PERIOD 10

class FinControl : public rclcpp::Node {
public:
  FinControl() : Node("fin_control") {

    // declare ros params
    this->declare_parameter("depth_kp", 0.0);
    this->declare_parameter("depth_ki", 0.0);
    this->declare_parameter("depth_kd", 0.0);
    this->declare_parameter("depth_min_output", 0);
    this->declare_parameter("depth_max_output", 0);
    this->declare_parameter("depth_bias", 0);
    this->declare_parameter("heading_kp", 0.0);
    this->declare_parameter("heading_ki", 0.0);
    this->declare_parameter("heading_kd", 0.0);
    this->declare_parameter("heading_min_output", 0);
    this->declare_parameter("heading_max_output", 0);
    this->declare_parameter("heading_bias", 0);
    this->declare_parameter("speed_kp", 0.0);
    this->declare_parameter("speed_ki", 0.0);
    this->declare_parameter("speed_kd", 0.0);
    this->declare_parameter("speed_min_output", 0);
    this->declare_parameter("speed_max_output", 0);
    this->declare_parameter("speed_bias", 0);

    // calibrate PID controllers
    myDepthPID.calibrate(this->get_parameter("depth_kp").as_double(),
                         this->get_parameter("depth_ki").as_double(),
                         this->get_parameter("depth_kd").as_double(),
                         this->get_parameter("depth_min_output").as_int(),
                         this->get_parameter("depth_max_output").as_int(),
                         PID_TIMER_PERIOD,
                         this->get_parameter("depth_bias").as_int());

    myHeadingPID.calibrate(this->get_parameter("heading_kp").as_double(),
                           this->get_parameter("heading_ki").as_double(),
                           this->get_parameter("heading_kd").as_double(),
                           this->get_parameter("heading_min_output").as_int(),
                           this->get_parameter("heading_max_output").as_int(),
                           PID_TIMER_PERIOD,
                           this->get_parameter("heading_bias").as_int());

    myVelocityPID.calibrate(this->get_parameter("speed_kp").as_double(),
                            this->get_parameter("speed_ki").as_double(),
                            this->get_parameter("speed_kd").as_double(),
                            this->get_parameter("speed_min_output").as_int(),
                            this->get_parameter("speed_max_output").as_int(),
                            PID_TIMER_PERIOD,
                            this->get_parameter("speed_bias").as_int());

    // declare ros publishers
    u_command_publisher_ =
        this->create_publisher<frost_interfaces::msg::UCommand>(
            "control_command", 10);

    // declare ros subscribers
    desired_depth_subscription_ =
        this->create_subscription<frost_interfaces::msg::DesiredDepth>(
            "desired_depth", 10,
            std::bind(&FinControl::desired_depth_callback, this, _1));

    desired_heading_subscription_ =
        this->create_subscription<frost_interfaces::msg::DesiredHeading>(
            "desired_heading", 10,
            std::bind(&FinControl::desired_heading_callback, this, _1));

    desired_speed_subscription_ =
        this->create_subscription<frost_interfaces::msg::DesiredSpeed>(
            "desired_speed", 10,
            std::bind(&FinControl::desired_speed_callback, this, _1));

    depth_subscription_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "depth_data", 10, std::bind(&FinControl::depth_callback, this, _1));

    velocity_subscription_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        "dvl_velocity", 10,
        std::bind(&FinControl::velocity_callback, this, _1));

    yaw_subscription_ = this->create_subscription<dvl_msgs::msg::DVLDR>(
        "dvl/position", 10, std::bind(&FinControl::yaw_callback, this, _1));

    // declare ros timers
    pid_timer_ = this->create_wall_timer(
        PID_TIMER_PERIOD_MS, std::bind(&FinControl::timer_callback, this));
  }

private:
  void
  desired_depth_callback(const frost_interfaces::msg::DesiredDepth &depth_msg) const {
    this->desired_depth_msg = depth_msg;
  }

  void desired_heading_callback(
      const frost_interfaces::msg::DesiredHeading &heading_msg) const {
    this->desired_heading_msg = heading_msg;
  }

  void
  desired_speed_callback(const frost_interfaces::msg::DesiredSpeed &speed_msg) const {
    this->desired_speed_msg = speed_msg;
  }

  void depth_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped &depth_msg) {
    this->depth = depth_msg.pose.pose.position.z;
  }

  void velocity_callback(
      const geometry_msgs::msg::TwistWithCovarianceStamped &velocity_msg) {
    this->x_velocity = velocity_msg.twist.twist.linear.x;
  }

  void yaw_callback(const dvl_msgs::msg::DVLDR &yaw_msg) { this->yaw = yaw_msg.yaw; }

  void timer_callback() {
    auto message = frost_interfaces::msg::UCommand();

    //////////////////////////////////////////////////////////
    // LOW-LEVEL CONTROLLER CODE STARTS HERE
    // - Reference wanted values using the desired_depth_msg,
    //   desired_heading_msg, and desired_speed_msg objects
    //////////////////////////////////////////////////////////

    // TODO: reset the dead reckoning on the dvl as soon as we start moving (?)

    int depth_pos = myDepthPID.compute(this->desired_depth_msg->desired_depth, depth);
    int heading_pos =
        myHeadingPID.compute(this->desired_heading_msg->desired_heading, yaw);
    int velocity_level =
        myVelocityPID.compute(this->desired_speed_msg->desired_speed, x_velocity);

    message.fin[0] = depth_pos;
    message.fin[1] = depth_pos; // TODO: counter-rotation offset?
    message.fin[2] = heading_pos;
    message.thruster = velocity_level;

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
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      depth_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::
      SharedPtr velocity_subscription_;
  rclcpp::Subscription<dvl_msgs::msg::DVLDR>::SharedPtr yaw_subscription_;

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