#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>  // For atan2, M_PI, fmin, fmax

#include "pid.cpp"
#include "frost_interfaces/msg/desired_depth.hpp"
#include "frost_interfaces/msg/desired_heading.hpp"
#include "frost_interfaces/msg/desired_speed.hpp"
#include "frost_interfaces/msg/u_command.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile);

/**
 * @brief A simple controls node.
 * @author Nelson Durrant
 * @date September 2024
 *
 * This node subscribes to desired depth, heading, and speed topics and actual
 * depth and heading topics. It then computes the control commands using various
 * controllers and publishes the control commands to a control command topic.
 *
 * Subscribes:
 * - init (std_msgs/msg/Empty)
 * - desired_depth (frost_interfaces/msg/DesiredDepth)
 * - desired_heading (frost_interfaces/msg/DesiredHeading)
 * - desired_speed (frost_interfaces/msg/DesiredSpeed)
 * - depth_data (geometry_msgs/msg/PoseWithCovarianceStamped)
 * - modem_status (seatrac_interfaces/msg/ModemStatus)
 * Publishes:
 * - controls/command (frost_interfaces/msg/UCommand)
 */
class CougControls : public rclcpp::Node {
public:
  /**
   * @brief Creates a new controls node.
   *
   * This constructor creates a new controls node with default values.
   */
  CougControls() : Node("coug_controls") {

    /**
     * @param timer_period
     *
     * The period of the control loop in milliseconds. The default value is
     * 80 ms from experimentation with the BlueRobotics depth sensor.
     */
    this->declare_parameter("timer_period", 80);

    /**
     * @param depth_kp
     *
     * The proportional constant for the depth PID controller. The default value
     * is 0.0.
     */
    this->declare_parameter("depth_kp", 0.0);

    /**
     * @param depth_ki
     *
     * The integral constant for the depth PID controller. The default value is
     * 0.0.
     */
    this->declare_parameter("depth_ki", 0.0);

    /**
     * @param depth_kd
     *
     * The derivative constant for the depth PID controller. The default value
     * is 0.0.
     */
    this->declare_parameter("depth_kd", 0.0);

    /**
     * @param depth_min_output
     *
     * The minimum output value for the depth PID controller. The default value
     * is 0.
     */
    this->declare_parameter("depth_min_output", 0);

    /**
     * @param depth_max_output
     *
     * The maximum output value for the depth PID controller. The default value
     * is 0.
     */
    this->declare_parameter("depth_max_output", 0);

    /**
     * @param depth_bias
     *
     * The bias value for the depth PID controller. The default value is 0.
     */
    this->declare_parameter("depth_bias", 0);

    /**
     * @param pitch_kp
     *
     * The proportional constant for the pitch PID controller. The default value
     * is 0.0.
     */
    this->declare_parameter("pitch_kp", 0.0);

    /**
     * @param pitch_ki
     * @param pitch_kp
     *
     * The proportional constant for the pitch PID controller. The default value
     * is 0.0.
     */
    this->declare_parameter("pitch_kp", 0.0);

    /**
     * @param pitch_ki
     *
     * The integral constant for the pitch PID controller. The default value is
     * 0.0.
     */
    this->declare_parameter("pitch_ki", 0.0);

    /**
     * @param pitch_kd
     *
     * The derivative constant for the pitch PID controller. The default value
     * is 0.0.
     * The integral constant for the pitch PID controller. The default value is
     * 0.0.
     */
    this->declare_parameter("pitch_ki", 0.0);

    /**
     * @param pitch_kd
     *
     * The derivative constant for the pitch PID controller. The default value
     * is 0.0.
     */
    this->declare_parameter("pitch_kd", 0.0);

    /**
     * @param pitch_min_output
     *
     * The minimum output value for the pitch PID controller. The default value
     * is 0.
     */
    this->declare_parameter("pitch_min_output", 0);
    this->declare_parameter("pitch_kd", 0.0);

    /**
     * @param pitch_min_output
     *
     * The minimum output value for the pitch PID controller. The default value
     * is 0.
     */
    this->declare_parameter("pitch_min_output", 0);

    /**
     * @param pitch_max_output
     *
     * The maximum output value for the pitch PID controller. The default value
     * is 0.
     */
    this->declare_parameter("pitch_max_output", 0);

    /**
     * @param pitch_bias
     * @param pitch_max_output
     *
     * The maximum output value for the pitch PID controller. The default value
     * is 0.
     */
    this->declare_parameter("pitch_max_output", 0);

    /**
     * @param pitch_bias
     *
     * The bias value for the pitch PID controller. The default value is 0.
     * The bias value for the pitch PID controller. The default value is 0.
     */
    this->declare_parameter("pitch_bias", 0);
    this->declare_parameter("pitch_bias", 0);

    /**
     * @param heading_kp
     *
     * The proportional constant for the heading PID controller. The default
     * value is 0.0.
     */
    this->declare_parameter("heading_kp", 0.0);

    /**
     * @param heading_ki
     *
     * The integral constant for the heading PID controller. The default value
     * is 0.0.
     */
    this->declare_parameter("heading_ki", 0.0);

    /**
     * @param heading_kd
     *
     * The derivative constant for the heading PID controller. The default value
     * is 0.0.
     */
    this->declare_parameter("heading_kd", 0.0);

    /**
     * @param heading_min_output
     *
     * The minimum output value for the heading PID controller. The default
     * value is 0.
     */
    this->declare_parameter("heading_min_output", 0);

    /**
     * @param heading_max_output
     *
     * The maximum output value for the heading PID controller. The default
     * value is 0.
     */
    this->declare_parameter("heading_max_output", 0);

    /**
     * @param heading_bias
     *
     * The bias value for the heading PID controller. The default value is 0.
     */
    this->declare_parameter("heading_bias", 0);


    /**
     * @param magnetic_declination
     * 
     * determines the offset to apply to the imu output based on location in degrees. 
     * The default is 10.7 degrees for Utah Lake
     */
    this->declare_parameter("magnetic_declination", 10.7);
    this->magnetic_declination = this->get_parameter("magnetic_declination").as_double();

    // calibrate PID controllers
    myDepthPID.calibrate(this->get_parameter("depth_kp").as_double(),
                         this->get_parameter("depth_ki").as_double(),
                         this->get_parameter("depth_kd").as_double(),
                         this->get_parameter("depth_min_output").as_int(),
                         this->get_parameter("depth_max_output").as_int(),
                         this->get_parameter("timer_period").as_int(),
                         this->get_parameter("depth_bias").as_int());

    myPitchPID.calibrate(this->get_parameter("pitch_kp").as_double(),
                         this->get_parameter("pitch_ki").as_double(),
                         this->get_parameter("pitch_kd").as_double(),
                         this->get_parameter("pitch_min_output").as_int(),
                         this->get_parameter("pitch_max_output").as_int(),
                         this->get_parameter("timer_period").as_int(),
                         this->get_parameter("pitch_bias").as_int());

    myPitchPID.calibrate(this->get_parameter("pitch_kp").as_double(),
                         this->get_parameter("pitch_ki").as_double(),
                         this->get_parameter("pitch_kd").as_double(),
                         this->get_parameter("pitch_min_output").as_int(),
                         this->get_parameter("pitch_max_output").as_int(),
                         this->get_parameter("timer_period").as_int(),
                         this->get_parameter("pitch_bias").as_int());

    myHeadingPID.calibrate(this->get_parameter("heading_kp").as_double(),
                           this->get_parameter("heading_ki").as_double(),
                           this->get_parameter("heading_kd").as_double(),
                           this->get_parameter("heading_min_output").as_int(),
                           this->get_parameter("heading_max_output").as_int(),
                           this->get_parameter("timer_period").as_int(),
                           this->get_parameter("heading_bias").as_int());

    /**
     * @brief Control command publisher.
     *
     * This publisher publishes the control commands to the "controls/command"
     * topic. It uses the UCommand message type.
     */
    u_command_publisher_ =
        this->create_publisher<frost_interfaces::msg::UCommand>(
            "controls/command", 10);

    /**
     * @brief Initialization subscriber.
     *
     * This subscriber subscribes to the "init" topic. It uses the Empty message
     * type.
     */
    init_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "init", 10, std::bind(&CougControls::init_callback, this, _1));

    /**
     * @brief Desired depth subscriber.
     *
     * This subscriber subscribes to the "desired_depth" topic. It uses the
     * DesiredDepth message type. 
     */
    desired_depth_subscription_ =
        this->create_subscription<frost_interfaces::msg::DesiredDepth>(
            "desired_depth", 10,
            std::bind(&CougControls::desired_depth_callback, this, _1));

    /**
     * @brief Desired heading subscriber.
     *
     * This subscriber subscribes to the "desired_heading" topic. It uses the
     * DesiredHeading message type. Expects the value to be in degrees from -180 to 180. ENU coordinate frame (0 being true east) 
     */
    desired_heading_subscription_ =
        this->create_subscription<frost_interfaces::msg::DesiredHeading>(
            "desired_heading", 10,
            std::bind(&CougControls::desired_heading_callback, this, _1));

    /**
     * @brief Desired speed subscriber.
     *
     * This subscriber subscribes to the "desired_speed" topic. It uses the
     * DesiredSpeed message type. Expected value from 0 to 100 (Non-dimensional)
     */
    desired_speed_subscription_ =
        this->create_subscription<frost_interfaces::msg::DesiredSpeed>(
            "desired_speed", 10,
            std::bind(&CougControls::desired_speed_callback, this, _1));

    /**
     * @brief Depth subscriber.
     *
     * This subscriber subscribes to the "depth_data" topic. It uses the
     * PoseWithCovarianceStamped message type. Expects data in ENU with the z value being more negative with increasing depth
     */
    actual_depth_subscription_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "depth_data", 10,
        std::bind(&CougControls::actual_depth_callback, this, _1));

    /**
     * @brief Yaw and Pitch subscriber.
     *
     * This subscriber subscribes to the "modem_imu" topic. It uses the Imu
     * message type.
     */
    actual_orientation_subscription_ =
        this->create_subscription<sensor_msgs::msg::Imu>(
            "modem_imu", 10,
            std::bind(&CougControls::actual_orientation_callback, this, _1));

    /**
     * @brief Control timer.
     *
     * This timer calls the control loop at the specified period.
     */
    controls_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->get_parameter("timer_period").as_int()),
        std::bind(&CougControls::timer_callback, this));
  }

private:
  /**
   * @brief Callback function for the init subscription.
   *
   * This method initializes the controls node by setting the init flag to
   * true.
   *
   * @param msg The Empty message recieved from the init topic.
   */
  void init_callback(const std_msgs::msg::Empty::SharedPtr msg) {

    (void)msg; // supress unused variable warning

    RCLCPP_INFO(this->get_logger(), "Init message recieved");
    this->init_flag = true;
  }

  /**
   * @brief Callback function for the desired_depth subscription.
   *
   * This method sets the desired depth value to the value received from the
   * desired depth message.
   *
   * @param depth_msg The DesiredDepth message recieved from the desired_depth
   * topic.
   */
  void
  desired_depth_callback(const frost_interfaces::msg::DesiredDepth &depth_msg) {
    this->desired_depth = depth_msg.desired_depth;
  }

  /**
   * @brief Callback function for the desired_heading subscription.
   *
   * This method sets the desired heading value to the value received from the
   * desired heading message. ENU yaw value -180 to 180
   *
   * @param heading_msg The DesiredHeading message recieved from the
   * desired_heading topic.
   */
  void desired_heading_callback(
      const frost_interfaces::msg::DesiredHeading &heading_msg) {
    this->desired_heading = heading_msg.desired_heading;
  }

  /**
   * @brief Callback function for the desired_speed subscription.
   *
   * This method sets the desired speed value to the value received from the
   * desired speed message.
   *
   * @param speed_msg The DesiredSpeed message recieved from the desired_speed
   * topic. The 
   */
  void
  desired_speed_callback(const frost_interfaces::msg::DesiredSpeed &speed_msg) {
    this->desired_speed = speed_msg.desired_speed;
  }

  /**
   * @brief Callback function for the depth subscription.
   *
   * This method sets the actual depth value to the value received from the
   * depth message. 
   *
   * @param depth_msg The PoseWithCovarianceStamped message recieved from the
   * depth_data topic.
   */
  void actual_depth_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped &depth_msg) {
    this->actual_depth = -depth_msg.pose.pose.position.z;
    //Negate the z value in ENU to get postive depth value
  }

  /**
   * @brief Callback function for the orientation subscription.
   *
   * This method sets the actual heading value to the value received from the
   * yaw message and the pitch value.  
   *
   * @param orientation_msg The Imu message recieved from the modem_imu topic.
   */
  void actual_orientation_callback(const sensor_msgs::msg::Imu &orientation_msg) {
    // Normalize and extract quaternion from the IMU message
    Eigen::Quaterniond q(
        orientation_msg.orientation.w,
        orientation_msg.orientation.x,
        orientation_msg.orientation.y,
        orientation_msg.orientation.z
    );
    q.normalize();
    
    this->current_quat = q;

    // // Convert quaternion to a 3x3 rotation matrix
    // Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();

    // // Extract Euler angles using ZYX order: yaw (Z), pitch (Y), roll (X)
    // Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);  // ZYX order

    // // Convert radians to degrees and center angles around 0
    // double yaw = euler_angles[0] * (180.0 / M_PI);
    // double pitch = euler_angles[1] * (180.0 / M_PI);
    // double roll = euler_angles[2] * (180.0 / M_PI);

    // // Normalize yaw, pitch, and roll to be within -180 to 180 degrees
    // if (yaw > 180.0) yaw -= 360.0;
    // else if (yaw < -180.0) yaw += 360.0;

    // if (pitch > 180.0) pitch -= 360.0;
    // else if (pitch < -180.0) pitch += 360.0;

    // if (roll > 180.0) roll -= 360.0;
    // else if (roll < -180.0) roll += 360.0;

    // // Store heading, pitch, and roll
    // this->actual_heading = yaw;
    // this->actual_pitch = pitch;
    // this->actual_roll = roll;

    // // Log the information
    // RCLCPP_INFO(this->get_logger(), "Yaw: %f, Pitch: %f, Roll: %f",
    //             this->actual_heading, this->actual_pitch, this->actual_roll);
  }


  /**
   * @brief Callback function for the PID control timer.
   *
   * This method computes the control commands using the PID controllers and
   * publishes the control commands to the controls/command topic.
   */

  double look_ahead_theta(double distance, double actual, double desired, double theta_max) {
    // Find the depth error
    double depth_error = desired - actual;

    // Calculate the angle using atan2 (rise over run: depth_error / distance)
    double theta_desired = atan2(depth_error, distance); // atan2(y, x)

    // Convert theta_desired from radians to degrees (optional, depending on your needs)
    theta_desired *= 180.0 / M_PI; // Uncomment if you want degrees instead of radians

    // Cap the desired theta within the range [-theta_max, theta_max]
    theta_desired = std::fmax(-theta_max, std::fmin(theta_desired, theta_max));

    return theta_desired;
  }

  double 

  void timer_callback() {
      auto message = frost_interfaces::msg::UCommand();
      message.header.stamp = this->now();

      if (this->init_flag) {

          double look_ahead = this->get_parameter("look_ahead").as_double();
          double theta_max = this->get_parameter("theta_max").as_double();



          // Calculate the desired pitch angle
          float theta_desired = myDepthPID.compute(this->desired_depth, this->actual_depth);
          RCLCPP_INFO(this->get_logger(), "[INFO] theta desired: %f, Actual Depth: %f, Desired Depth: %f", float(theta_desired), float(this->actual_depth), float(this->desired_depth));

          // Step 1: Create the target quaternion from desired pitch and heading
          Eigen::Quaterniond target_quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                                          Eigen::AngleAxisd(theta_desired * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(this->desired_heading * M_PI / 180.0, Eigen::Vector3d::UnitZ());

          // Step 2: Compute the quaternion error directly
          Eigen::Quaterniond q_err = target_quat * this->current_quat.inverse();

          // Step 3: Extract pitch and yaw error directly from the quaternion error vector part
          Eigen::Vector3d error_vec = q_err.vec();

          // Convert the error vector’s Y and Z components to pitch and yaw errors (proportional to pitch and yaw deviations)
          double pitch_err = 2.0 * error_vec.y() * 180.0 / M_PI;  // Scaled by 2 and converted to degrees
          double yaw_err = 2.0 * error_vec.z() * 180.0 / M_PI;

          RCLCPP_INFO(this->get_logger(), "Yaw Error: %f, Pitch Error: %f", yaw_err, pitch_err);

          // Step 4: Apply PID control to pitch and heading errors directly
          int depth_pos = (int)myPitchPID.compute(0, pitch_err);  // No additional scaling needed
          int heading_pos = (int)myHeadingPID.compute(0, yaw_err);

          // Step 5: Set fin positions and publish the command
          message.fin[0] = heading_pos;    // top fin
          message.fin[1] = depth_pos;      // right fin
          message.fin[2] = depth_pos;      // left fin
          message.thruster = this->desired_speed;

          u_command_publisher_->publish(message);
      }
  }


  // micro-ROS objects
  rclcpp::TimerBase::SharedPtr controls_timer_;
  rclcpp::Publisher<frost_interfaces::msg::UCommand>::SharedPtr
      u_command_publisher_;
  rclcpp::Subscription<frost_interfaces::msg::DesiredDepth>::SharedPtr
      desired_depth_subscription_;
  rclcpp::Subscription<frost_interfaces::msg::DesiredHeading>::SharedPtr
      desired_heading_subscription_;
  rclcpp::Subscription<frost_interfaces::msg::DesiredSpeed>::SharedPtr
      desired_speed_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      actual_depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
      actual_orientation_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr init_subscription_;

  // node initialization flag
  bool init_flag = false;

  // control objects
  PID myHeadingPID;
  PID myDepthPID;
  PID myPitchPID;
  PID myPitchPID;

  // magnetic declination parameter
  double magnetic_declination;

  // node desired values
  float desired_depth = 0.0;
  float desired_heading = 0.0;
  float desired_speed = 0.0;

  // node actual values
  float actual_depth = 0.0;
  Eigen::Quaterniond current_quat;
  // float actual_pitch = 0.0;
  // float actual_roll = 0.0;
  // float actual_heading = 0.0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CougControls>());
  rclcpp::shutdown();
  return 0;
}
