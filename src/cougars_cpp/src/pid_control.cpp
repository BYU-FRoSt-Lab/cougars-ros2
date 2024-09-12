#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cougars_cpp/pid.h"
#include "frost_interfaces/msg/desired_depth.hpp"
#include "frost_interfaces/msg/desired_heading.hpp"
#include "frost_interfaces/msg/desired_speed.hpp"
#include "frost_interfaces/msg/modem_rec.hpp"
#include "frost_interfaces/msg/u_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile);

class PIDControl : public rclcpp::Node {
public:
  PIDControl() : Node("pid_control") {

    // declare ros params
    this->declare_parameter("pid_timer_period",
                            80); // from experimentation with depth sensor
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
                         this->get_parameter("pid_timer_period").as_int(),
                         this->get_parameter("depth_bias").as_int());

    myHeadingPID.calibrate(this->get_parameter("heading_kp").as_double(),
                           this->get_parameter("heading_ki").as_double(),
                           this->get_parameter("heading_kd").as_double(),
                           this->get_parameter("heading_min_output").as_int(),
                           this->get_parameter("heading_max_output").as_int(),
                           this->get_parameter("pid_timer_period").as_int(),
                           this->get_parameter("heading_bias").as_int());

    myVelocityPID.calibrate(this->get_parameter("speed_kp").as_double(),
                            this->get_parameter("speed_ki").as_double(),
                            this->get_parameter("speed_kd").as_double(),
                            this->get_parameter("speed_min_output").as_int(),
                            this->get_parameter("speed_max_output").as_int(),
                            this->get_parameter("pid_timer_period").as_int(),
                            this->get_parameter("speed_bias").as_int());

    // declare ros publishers
    u_command_publisher_ =
        this->create_publisher<frost_interfaces::msg::UCommand>(
            "control_command", 10);

    // declare ros subscribers
    init_subscription_ = 
        this->create_subscription<std_msgs::msg::Empty>(
            "init", 10,
            std::bind(&PIDControl::init_callback, this, _1));

    desired_depth_subscription_ =
        this->create_subscription<frost_interfaces::msg::DesiredDepth>(
            "desired_depth", 10,
            std::bind(&PIDControl::desired_depth_callback, this, _1));

    desired_heading_subscription_ =
        this->create_subscription<frost_interfaces::msg::DesiredHeading>(
            "desired_heading", 10,
            std::bind(&PIDControl::desired_heading_callback, this, _1));

    desired_speed_subscription_ =
        this->create_subscription<frost_interfaces::msg::DesiredSpeed>(
            "desired_speed", 10,
            std::bind(&PIDControl::desired_speed_callback, this, _1));

    depth_subscription_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "depth_data", 10, std::bind(&PIDControl::depth_callback, this, _1));

    yaw_subscription_ =
        this->create_subscription<frost_interfaces::msg::ModemRec>(
            "modem_rec", 10, std::bind(&PIDControl::yaw_callback, this, _1));

    // declare ros timers
    pid_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            this->get_parameter("pid_timer_period").as_int()),
        std::bind(&PIDControl::timer_callback, this));
  }

private:
  void
  init_callback(const std_msgs::msg::Empty::SharedPtr msg) {
    this->init_flag = true;
  }

  desired_depth_callback(const frost_interfaces::msg::DesiredDepth &depth_msg) {
    this->desired_depth = depth_msg.desired_depth;
  }

  void desired_heading_callback(
      const frost_interfaces::msg::DesiredHeading &heading_msg) {
    this->desired_heading = heading_msg.desired_heading;
  }

  void
  desired_speed_callback(const frost_interfaces::msg::DesiredSpeed &speed_msg) {
    this->desired_speed = speed_msg.desired_speed;
  }

  void depth_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped &depth_msg) {
    this->depth = depth_msg.pose.pose.position.z;
  }

  void yaw_callback(const frost_interfaces::msg::ModemRec &yaw_msg) {

    // Check if the message is a status message
    if (yaw_msg.msg_id == 0x10) {
      this->yaw = yaw_msg.attitude_yaw;
    }
  }

  void timer_callback() {
    auto message = frost_interfaces::msg::UCommand();
    message.header.stamp = this->now();

    if (this->init_flag) {

      //////////////////////////////////////////////////////////
      // LOW-LEVEL CONTROLLER CODE STARTS HERE
      // - Reference wanted values using the desired_depth_msg,
      //   desired_heading_msg, and desired_speed_msg objects
      //////////////////////////////////////////////////////////

      int depth_pos = myDepthPID.compute(this->desired_depth, -depth);
      int heading_pos = myHeadingPID.compute(this->desired_heading, yaw);
      int velocity_level =
          this->desired_speed; // myVelocityPID.compute(this->desired_speed,
                              // x_velocity);

      message.fin[0] = heading_pos;
      message.fin[1] = depth_pos; // TODO: counter-rotation offset?
      message.fin[2] = depth_pos;
      message.thruster = velocity_level;

      u_command_publisher_->publish(message);

      RCLCPP_INFO(this->get_logger(),
                  "Bottom Servos: %d, Top Servo: %d, Thruster: %d", depth_pos,
                  heading_pos, velocity_level);

      //////////////////////////////////////////////////////////
      // LOW-LEVEL CONTROLLER CODE ENDS HERE
      //////////////////////////////////////////////////////////
    }
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
  rclcpp::Subscription<frost_interfaces::msg::ModemRec>::SharedPtr
      yaw_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr init_subscription_;

  // flags
  bool init_flag = false;

  // class desired value variables
  float desired_depth = 0.0;
  float desired_heading = 0.0;
  float desired_speed = 0.0;

  // control objects
  PID myHeadingPID;
  PID myDepthPID;
  PID myVelocityPID;

  // class sensor variables
  float yaw = 0.0;
  float x_velocity = 0.0;
  float depth = 0.0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControl>());
  rclcpp::shutdown();
  return 0;
}
