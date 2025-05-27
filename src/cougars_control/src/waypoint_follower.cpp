#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "frost_interfaces/msg/desired_depth.hpp"
#include "frost_interfaces/msg/desired_heading.hpp"
#include "frost_interfaces/msg/desired_speed.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "yaml-cpp/yaml.h" // For YAML parsing
#include <fstream> 
#include <sstream> 

// Define a structure to hold waypoint information
struct Waypoint {
    int id;
    double enu_x;
    double enu_y;
    double depth; // Target depth at this waypoint
};

// Define a structure for LLA coordinates (primarily for origin, if needed later)
struct LLA {
    double latitude;
    double longitude;
    double altitude;
};

class WaypointFollowerNode : public rclcpp::Node
{
public:
    WaypointFollowerNode() : Node("waypoint_follower"), current_waypoint_index_(0), mission_loaded_(false), mission_active_(false)
    {
        // Declare parameters
        this->declare_parameter<std::string>("waypoint_file_path", "mission.yaml"); //fix this yaml file implementation
        this->declare_parameter<double>("slip_radius", 2.0); // meters
        this->declare_parameter<double>("desired_travel_speed", 20.0); // Non-dimensional, e.g., 0-100
        this->declare_parameter<double>("loop_rate", 10.0); // Hz

        // Get parameters
        waypoint_file_path_ = this->get_parameter("waypoint_file_path").as_string();
        slip_radius_ = this->get_parameter("slip_radius").as_double();
        desired_travel_speed_ = this->get_parameter("desired_travel_speed").as_double();
        loop_rate_ = this->get_parameter("loop_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "Waypoint file path: %s", waypoint_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Slip radius: %.2f m", slip_radius_);
        RCLCPP_INFO(this->get_logger(), "Desired travel speed: %.2f", desired_travel_speed_);

        // Publishers
        desired_depth_pub_ = this->create_publisher<frost_interfaces::msg::DesiredDepth>("desired_depth", 10);
        desired_heading_pub_ = this->create_publisher<frost_interfaces::msg::DesiredHeading>("desired_heading", 10);
        desired_speed_pub_ = this->create_publisher<frost_interfaces::msg::DesiredSpeed>("desired_speed", 10);

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "smoothed_output", 10, std::bind(&WaypointFollowerNode::odom_callback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "modem_imu", 10, std::bind(&WaypointFollowerNode::imu_callback, this, std::placeholders::_1));

        // Load waypoints
        if (load_waypoints()) {
            mission_loaded_ = true;
            mission_active_ = true; // Start mission immediately after loading
            RCLCPP_INFO(this->get_logger(), "Mission loaded successfully with %zu waypoints.", waypoints_.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mission from %s", waypoint_file_path_.c_str());
        }

        // Main control loop timer
        control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / loop_rate_)),
            std::bind(&WaypointFollowerNode::control_loop_callback, this));
        
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_heading_degrees_ = 0.0;
    }

private:
    // Callback for odometry data (current position)
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        // current_z_ = msg->pose.pose.position.z; // If needed
    }

    // Callback for IMU data (current heading)
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // Roll, Pitch, Yaw in radians

        // Convert yaw to degrees ENU (0 East, 90 North)
        // Assuming IMU provides yaw where 0 is North and positive is East (NED)
        // Or if IMU is already in ENU frame with Z up, yaw might be around Z axis.
        // The coug_controls.cpp expects ENU heading (-180 to 180, 0 is East)
        // Let's assume yaw from IMU is standard ROS: 0 along X-axis (forward), positive CCW.
        // If IMU gives yaw relative to North (NED frame), conversion is needed.
        // For simplicity, let's assume the IMU provides yaw in radians where 0 is East.
        // This part is CRITICAL and depends on your actual IMU setup and TF tree.
        // The `coug_controls.cpp` converts quaternion to Euler (Z, Y, X) and uses Z-axis rotation as yaw.
        // It then expects this yaw to be in degrees, -180 to 180, ENU.
        
        current_heading_degrees_ = normalize_angle_degrees(yaw * 180.0 / M_PI); 
        // RCLCPP_DEBUG(this->get_logger(), "Current Heading (Degrees ENU): %.2f", current_heading_degrees_);
    }

    // Main control loop
    void control_loop_callback()
    {

      // Add emergency stop service here
        if (!mission_loaded_ || !mission_active_ || waypoints_.empty()) {
            if (!mission_loaded_) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Mission not loaded.");
            else if (waypoints_.empty()) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No waypoints in mission.");
            else if (!mission_active_) RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Mission complete or not active.");
            // Optionally, publish zero speed/hold position commands
            publish_control_commands(current_heading_degrees_, waypoints_.empty() ? 0.5 : waypoints_.back().depth, 0.0); // Hold last depth, zero speed
            return;
        }

        if (current_waypoint_index_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached. Mission complete.");
            mission_active_ = false;
            // Publish zero speed, maintain last depth
            publish_control_commands(current_heading_degrees_, waypoints_.back().depth, 0.0);
            return;
        }

        const auto& target_wp = waypoints_[current_waypoint_index_];
        double distance_to_target = calculate_distance(current_x_, current_y_, target_wp.enu_x, target_wp.enu_y);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "Current Pos: (%.2f, %.2f), Target WP %d: (%.2f, %.2f), Dist: %.2f, Slip Radius: %.2f",
                       current_x_, current_y_, target_wp.id, target_wp.enu_x, target_wp.enu_y, distance_to_target, slip_radius_);

        if (distance_to_target < slip_radius_) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %d (%.2fm < %.2fm). Moving to next.",
                        target_wp.id, distance_to_target, slip_radius_);
            current_waypoint_index_++;
            if (current_waypoint_index_ >= waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), "Last waypoint reached. Mission complete.");
                mission_active_ = false;
                publish_control_commands(current_heading_degrees_, target_wp.depth, 0.0); // Hold last depth, zero speed
                return;
            }
            // Re-evaluate immediately for the new waypoint in the next loop iteration
             RCLCPP_INFO(this->get_logger(), "Next waypoint ID: %d", waypoints_[current_waypoint_index_].id);
        }
        
        // If mission still active (either not at current WP or just advanced to a new one)
        if (mission_active_ && current_waypoint_index_ < waypoints_.size()) {
            const auto& next_target_wp = waypoints_[current_waypoint_index_];
            double desired_heading_rad = calculate_bearing(current_x_, current_y_, next_target_wp.enu_x, next_target_wp.enu_y);
            double desired_heading_deg = normalize_angle_degrees(desired_heading_rad * 180.0 / M_PI);
            
            publish_control_commands(desired_heading_deg, next_target_wp.depth, desired_travel_speed_);
             RCLCPP_DEBUG(this->get_logger(), "Publishing: Desired Heading: %.2f deg, Desired Depth: %.2f m, Desired Speed: %.2f",
                         desired_heading_deg, next_target_wp.depth, desired_travel_speed_);
        }
    }

    // Publish control commands
    void publish_control_commands(double heading_deg, double depth, double speed)
    {
        auto depth_msg = frost_interfaces::msg::DesiredDepth();
        depth_msg.desired_depth = static_cast<float>(depth);
        desired_depth_pub_->publish(depth_msg);

        auto heading_msg = frost_interfaces::msg::DesiredHeading();
        heading_msg.desired_heading = static_cast<float>(heading_deg);
        desired_heading_pub_->publish(heading_msg);

        auto speed_msg = frost_interfaces::msg::DesiredSpeed();
        speed_msg.desired_speed = static_cast<float>(speed);
        desired_speed_pub_->publish(speed_msg);
    }

    // Load waypoints from YAML file
    bool load_waypoints()
    {
    
        // // =======================================================
        // //                  START DEBUG CODE
        // // =======================================================
        // RCLCPP_INFO(this->get_logger(), "Attempting to load waypoints from: %s", waypoint_file_path_.c_str());
        // std::ifstream file_stream(waypoint_file_path_);

        // if (!file_stream.is_open()) {
        //     RCLCPP_ERROR(this->get_logger(), "DEBUG: FAILED TO OPEN FILE STREAM FOR: %s", waypoint_file_path_.c_str());
        //     RCLCPP_ERROR(this->get_logger(), "DEBUG: Check if the file exists at this path in your 'install' directory and check permissions.");
        //     return false;
        // }

        // std::stringstream buffer;
        // buffer << file_stream.rdbuf();
        // std::string file_content = buffer.str();
        // file_stream.close();

        // if (file_content.empty()) {
        //     RCLCPP_ERROR(this->get_logger(), "DEBUG: File is EMPTY or could not be read: %s", waypoint_file_path_.c_str());
        //     return false;
        // }

        // RCLCPP_INFO(this->get_logger(), "DEBUG: --- Start of YAML Content (Read Successfully) ---");
        // RCLCPP_INFO(this->get_logger(), "\n%s", file_content.c_str());
        // RCLCPP_INFO(this->get_logger(), "DEBUG: --- End of YAML Content ---");
        // // =======================================================
        // //                   END DEBUG CODE
        // // =======================================================

        try {
            YAML::Node mission_yaml = YAML::LoadFile(waypoint_file_path_);

            if (!mission_yaml["origin_lla"] || !mission_yaml["waypoints"]) {
                RCLCPP_ERROR(this->get_logger(), "YAML file missing 'origin_lla' or 'waypoints' section.");
                return false;
            }
            
            // Optionally use origin_lla if needed for transformations later
            // origin_lla_.latitude = mission_yaml["origin_lla"]["latitude"].as<double>();
            // origin_lla_.longitude = mission_yaml["origin_lla"]["longitude"].as<double>();
            // origin_lla_.altitude = mission_yaml["origin_lla"]["altitude"].as<double>();
            // RCLCPP_INFO(this->get_logger(), "Origin LLA: Lat %.6f, Lon %.6f, Alt %.2f",
            //             origin_lla_.latitude, origin_lla_.longitude, origin_lla_.altitude);


            const YAML::Node& wps = mission_yaml["waypoints"];
            if (!wps.IsSequence()) {
                 RCLCPP_ERROR(this->get_logger(), "'waypoints' is not a sequence in YAML file.");
                return false;
            }

            waypoints_.clear();
            for (const auto& wp_node : wps) {
                Waypoint wp;
                wp.id = wp_node["id"].as<int>();
                wp.enu_x = wp_node["position_enu"]["x"].as<double>();
                wp.enu_y = wp_node["position_enu"]["y"].as<double>();
                // YAML has 'z' for ENU, but we typically control 'depth'.
                // If z is height above origin, depth = -z (assuming origin is at surface)
                // Or, use the explicit 'depth' field if available.
                wp.depth = wp_node["depth"].as<double>(); 
                waypoints_.push_back(wp);
                RCLCPP_INFO(this->get_logger(), "Loaded WP ID: %d, ENU_X: %.2f, ENU_Y: %.2f, Depth: %.2f",
                            wp.id, wp.enu_x, wp.enu_y, wp.depth);
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
            return false;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading waypoints: %s", e.what());
            return false;
        }
        return !waypoints_.empty();
    }

    // Calculate 2D Euclidean distance
    double calculate_distance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }

    // Calculate bearing (angle) from (x1, y1) to (x2, y2) in radians
    // Result is in ENU: 0 rad is East, PI/2 rad is North
    double calculate_bearing(double x1, double y1, double x2, double y2)
    {
        return std::atan2(y2 - y1, x2 - x1);
    }

    // Normalize angle to be within -180 to 180 degrees
    double normalize_angle_degrees(double angle_deg) {
        angle_deg = fmod(angle_deg + 180.0, 360.0);
        if (angle_deg < 0) {
            angle_deg += 360.0;
        }
        return angle_deg - 180.0;
    }

    // Member variables
    rclcpp::Publisher<frost_interfaces::msg::DesiredDepth>::SharedPtr desired_depth_pub_;
    rclcpp::Publisher<frost_interfaces::msg::DesiredHeading>::SharedPtr desired_heading_pub_;
    rclcpp::Publisher<frost_interfaces::msg::DesiredSpeed>::SharedPtr desired_speed_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    std::string waypoint_file_path_;
    double slip_radius_;
    double desired_travel_speed_;
    double loop_rate_;

    std::vector<Waypoint> waypoints_;
    LLA origin_lla_;
    size_t current_waypoint_index_;
    bool mission_loaded_;
    bool mission_active_;

    double current_x_;
    double current_y_;
    // double current_z_; // From odometry if needed
    double current_heading_degrees_; // ENU, 0 is East
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // std::make_shared<WaypointFollowerNode>(); // Node will spin itself if needed or rely on executor
    rclcpp::spin(std::make_shared<WaypointFollowerNode>()); 
    rclcpp::shutdown();
    return 0;
}
