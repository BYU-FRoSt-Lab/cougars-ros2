#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>  // For atan2, M_PI, fmin, fmax

#include "pid.cpp"
#include "actuator.cpp"
// #include "pid_int2.cpp"
#include "frost_interfaces/msg/desired_depth.hpp"
#include "frost_interfaces/msg/desired_heading.hpp"
#include "frost_interfaces/msg/desired_speed.hpp"
#include "frost_interfaces/msg/u_command.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

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
 * Service:
 * - init_controls (std_srvs/srv/SetBool) 
 *
 * Subscribes:
 * - desired_depth (frost_interfaces/msg/DesiredDepth)
 * - desired_heading (frost_interfaces/msg/DesiredHeading)
 * - desired_speed (frost_interfaces/msg/DesiredSpeed)
 * - depth_data (geometry_msgs/msg/PoseWithCovarianceStamped)
 * - modem_status (seatrac_interfaces/msg/ModemStatus)
 * 
 * Publishes:
 * - controls/command (frost_interfaces/msg/UCommand)
 */
class CougControls : public rclcpp::Node {
public:

  /**
   * Creates a new controls node.
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
    this->declare_parameter("depth_min_output", 0.0);

    /**
     * @param depth_max_output
     *
     * The maximum output value for the depth PID controller. The default value
     * is 0.
     */
    this->declare_parameter("depth_max_output", 0.0);

    /**
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
     */
    this->declare_parameter("pitch_kd", 0.0);

    /**
     * @param pitch_min_output
     *
     * The minimum output value for the pitch PID controller. The default value
     * is 0.
     */
    this->declare_parameter("pitch_min_output", 0.0);

    /**
     * @param pitch_max_output
     *
     * The maximum output value for the pitch PID controller. The default value
     * is 0.
     */
    this->declare_parameter("pitch_max_output", 0.0);

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
    this->declare_parameter("heading_min_output", 0.0);

    /**
     * @param heading_max_output
     *
     * The maximum output value for the heading PID controller. The default
     * value is 0.
     */
    this->declare_parameter("heading_max_output", 0.0);

    /**
     * @param magnetic_declination
     * 
     * determines the offset to apply to the imu output based on location in degrees. 
     * The default is 10.7 degrees for Utah Lake
     */
    this->declare_parameter("magnetic_declination", 10.7);
    this->magnetic_declination = this->get_parameter("magnetic_declination").as_double();
    this->declare_parameter("surge_threshold", -1.0);
    this->declare_parameter("wn_d_z", 0.09);
    this->declare_parameter("wn_d_theta", 0.25);
    this->declare_parameter("outer_loop_threshold", 2.5);
    this->declare_parameter("saturation_offset", 1.7);

    // calibrate PID controllers
    myDepthPID.initialize(this->get_parameter("depth_kp").as_double(),
                         this->get_parameter("depth_ki").as_double(),
                         this->get_parameter("depth_kd").as_double(),
                         this->get_parameter("depth_min_output").as_double(),         
                         this->get_parameter("depth_max_output").as_double(),
                         (float)this->get_parameter("timer_period").as_int()); 

    // TODO parameterize this if it works
    float scalar = 0.5 * 997 * 0.00665 * 0.5 * -0.43 * std::cos(30 * (M_PI / 180.0));

    myPitchPID.initialize(this->get_parameter("pitch_kp").as_double(),
                         this->get_parameter("pitch_ki").as_double(),
                         this->get_parameter("pitch_kd").as_double(),
                         this->get_parameter("pitch_min_output").as_double(),       //THIS is the fin min and max
                         this->get_parameter("pitch_max_output").as_double(),
                         (float)this->get_parameter("timer_period").as_int(),
                         scalar);

    myHeadingPID.initialize(this->get_parameter("heading_kp").as_double(),
                           this->get_parameter("heading_ki").as_double(),
                           this->get_parameter("heading_kd").as_double(),
                           this->get_parameter("heading_min_output").as_double(),
                           this->get_parameter("heading_max_output").as_double(),
                           (float)this->get_parameter("timer_period").as_int());

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
     * @brief Initialization Service.
     *
     * This service "init_controls" topic. It uses the SetBool service
     * type.
     */
    init_service_ = this->create_service<std_srvs::srv::SetBool>(
            "init_controls",
            std::bind(&CougControls::handle_service, this, std::placeholders::_1, std::placeholders::_2));

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
     * @brief Converted DVL velocity subscriber.
     *
     * This subscriber subscribes to the "dvl/velocity" topic. It uses the TwistWithCovarianceStamped
     * message type.
     */
    actual_velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "dvl/velocity", qos,
        std::bind(&CougControls::dvl_velocity_callback, this, _1));

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
   * @brief Callback function for the initialization service.
   *
   * This method initializes the controls node by setting the init flag to
   * true.
   */
  void handle_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    // Set the initialization flag based on the request
    this->init_flag = request->data;

    // Respond with success and an appropriate message
    if (request->data) {
        response->success = true;
        response->message = "Controller Node initialized.";
    } else {
        response->success = false;
        response->message = "Controller Node de-initialized.";
    }
    RCLCPP_INFO(this->get_logger(), "Init Service recieved");
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
    if (depth_msg.desired_depth == this->desired_depth){
        return;
    }
    this->desired_depth = depth_msg.desired_depth;

    // When a new desired depth sent update the depth_ref to the actual depth
    this->depth_ref = this->actual_depth;
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
   * @brief Callback function for the velocity subscription.
   *
   * This method sets the velocity value to the value received from the
   * velocity message. 
   *
   * @param velocity The TwistWithCovarianceStamped message recieved from the
   * dvl/velocity topic.
   */
  void dvl_velocity_callback(
      const geometry_msgs::msg::TwistWithCovarianceStamped &velocity_msg) {
    
    //This is taking from the converted dvl velocity only when dvl velocity is valid.
    // Velocity in x is expected to be 0.67 m/s at 20/100 thruster with velocity in y and z expected to be < 0.01
    this->velocity[0] = velocity_msg.twist.twist.linear.x;
    this->velocity[1] = velocity_msg.twist.twist.linear.y;
    this->velocity[2] = velocity_msg.twist.twist.linear.z;

  }


  void normalizeAngles(double& yaw, double& pitch, double& roll) {
    // Normalize yaw to [-180, 180]
    yaw = fmod(yaw + 180.0, 360.0) - 180.0;

    // Limit pitch to [-90, 90]
    if (pitch > 90.0) {
        pitch = 180.0 - pitch;
        yaw += 180.0;
        roll += 180.0;
    } else if (pitch < -90.0) {
        pitch = -180.0 - pitch;
        yaw += 180.0;
        roll += 180.0;
    }

    // Normalize roll to [-180, 180]
    roll = fmod(roll + 180.0, 360.0) - 180.0;

    // Ensure yaw is still in [-180, 180] after adjustments
    yaw = fmod(yaw + 180.0, 360.0) - 180.0;
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

    // Convert quaternion to a 3x3 rotation matrix
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();

    // Extract Euler angles using ZYX order: yaw (Z), pitch (Y), roll (X)
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);  // ZYX order

    // Convert radians to degrees and center angles around 0
    double yaw = euler_angles[0] * (180.0 / M_PI);
    double pitch = euler_angles[1] * (180.0 / M_PI);
    double roll = euler_angles[2] * (180.0 / M_PI);

    // std::cout << "Before normalization:" << std::endl;
    // std::cout << "Yaw: " << yaw << ", Pitch: " << pitch << ", Roll: " << roll << std::endl;

    normalizeAngles(yaw, pitch, roll);

    // std::cout << "After normalization:" << std::endl;
    // std::cout << "Yaw: " << yaw << ", Pitch: " << pitch << ", Roll: " << roll << std::endl;

    // Store heading, pitch, and roll
    this->actual_heading = yaw;
    // std::cout << "actual heading: " << this->actual_heading << std::endl;
    this->actual_pitch = pitch;
    this->actual_roll = roll;

    this->pitch_rate = orientation_msg.angular_velocity.y;

    // // // Log the information
    // RCLCPP_INFO(this->get_logger(), "Yaw: %f, Pitch: %f, Roll: %f",
    //             this->actual_heading, this->actual_pitch, this->actual_roll);
  }

  float calculateYawError(float desired_heading, float actual_heading) {
    float yaw_err = desired_heading - actual_heading;
    
    // Normalize the error to the range [-180, 180]
    if (yaw_err > 180.0f) {
        yaw_err -= 360.0f;
    } else if (yaw_err < -180.0f) {
        yaw_err += 360.0f;
    }
    
    return yaw_err;
  }
  
  int depth_autopilot(float depth, float depth_d){
    float surge = this->velocity[0];
    float surge_threshold = this->get_parameter("surge_threshold").as_double();
    float timer_period = this->get_parameter("timer_period").as_int() / 1000.0;
    float theta_max = this->get_parameter("depth_max_output").as_double();  //FIGURE OUT THIS PARAM for the vehicle at full fin 
    float wn_d_z = this->get_parameter("wn_d_z").as_double(); //TODO this is an important parameter - See how z tracks z ref
    float wn_d_theta = this->get_parameter("wn_d_theta").as_double();
    float outer_loop_threshold = this->get_parameter("outer_loop_threshold").as_double();
    float saturation_offset = this->get_parameter("saturation_offset").as_double();

    if(surge < surge_threshold){
        this->theta_ref = this->actual_pitch;   
        return 0;
    }
    // Low Pass filter for depth reference
    this->depth_ref = std::exp(-timer_period * wn_d_z) * this->depth_ref
                + (1 - std::exp(-timer_period * wn_d_z)) * depth_d;

    double saturated = saturation_offset * std::copysign(1.0, depth_d - depth);  //CHECK THE SIGN ON THIS
    
    if (std::abs(depth_d - depth) > outer_loop_threshold) {
        // saturate theta_d
        float theta_d = theta_max * std::copysign(1.0, depth_d - depth);
        this->theta_ref = std::exp(-timer_period * wn_d_theta) * this->theta_ref
                        + (1.0 - std::exp(-timer_period * wn_d_theta)) * theta_d;

        this->depth_ref = depth + saturated; // Override low pass filter input
        std::cout <<  "Saturated Depth PID mode" << std::endl;
        myDepthPID.reset_int();
    } else {
        this->theta_ref = myDepthPID.compute(this->depth_ref, this->actual_depth, this->velocity[2]);
    }

    std::cout <<  "Depth: " << this->actual_depth << " Depth Ref: " << this->depth_ref << " Desired Depth: " << depth_d << std::endl;


    std::cout <<  "Pitch: " << this->actual_pitch << " Desired Pitch: " << this->theta_ref  << " Pitch Rate: " << this->pitch_rate << std::endl;
    int depth_pos = (int)myPitchPID.compute(this->theta_ref, this->actual_pitch, this->pitch_rate, this->velocity[0]);  


    // Add torque equilibruim?
    // int depth_pos = (int)calculate_deflection(torque_s, this->velocity, -0.43, deltaMax);
    //TODO calculate beforehand what torque creates a max deflection.

    std::cout << " Fin: " << depth_pos << std::endl;

    return depth_pos;
  }

  /**
   * @brief Callback function for the PID control timer.
   *
   * This method computes the control commands using the PID controllers and
   * publishes the control commands to the controls/command topic.
   */
  void timer_callback() {
      auto message = frost_interfaces::msg::UCommand();
      message.header.stamp = this->now();

      if (this->init_flag) {
          int depth_pos = depth_autopilot(this->actual_depth, this->desired_depth);
          
          // Handling roll over when taking the error difference
          // given desired heading and actual heading from -180 to 180
          // if they are both negative or they are both positive than just take the difference
          float yaw_err = calculateYawError(this->desired_heading, this->actual_heading);
          // // Log the information
          
          int heading_pos = (int)myHeadingPID.compute(0.0, yaw_err);
          std::cout <<  "Yaw: " << this->actual_heading << "Desired Yaw: " << this->desired_heading << "Yaw Error: " << yaw_err << std::endl;

          // Step 5: Set fin positions and publish the command
          message.fin[0] = heading_pos;    // top fin
          message.fin[1] = -depth_pos;      // starboard side fin
          message.fin[2] = depth_pos;      // port side fin
          message.thruster = this->desired_speed;
        
          u_command_publisher_->publish(message);
      }
      else{
          message.fin[0] = 0;    // top fin
          message.fin[1] = 0;      // right fin
          message.fin[2] = 0;      // left fin
          message.thruster = 0;

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
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      actual_velocity_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
      actual_orientation_subscription_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr init_service_;

  // node initialization flag
  bool init_flag = false;

  // control objects
  PID myHeadingPID;
  PID myDepthPID;
  PID myPitchPID;

  // magnetic declination parameter
  double magnetic_declination;

  // node desired values
  float desired_depth = 0.0;
  float desired_heading = 0.0;
  float desired_speed = 0.0;

  float depth_ref;
  float theta_ref;

  // node actual values
  float actual_depth = 0.0;
  Eigen::Quaterniond current_quat;
  float actual_pitch = 0.0;
  float actual_roll = 0.0;
  float actual_heading = 0.0;

  float pitch_rate;
  float velocity[3];
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CougControls>());
  rclcpp::shutdown();
  return 0;
}
