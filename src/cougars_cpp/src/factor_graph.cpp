#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;

class FactorStateNode : public rclcpp::Node
{
public:
    FactorStateNode()
        : Node("factor_state_node")
    {
        // Set up the subscriptions
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/modem_imu", 10,
            std::bind(&FactorStateNode::imuCallback, this, std::placeholders::_1));

        gps_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/gps/odom", 10,
            std::bind(&FactorStateNode::gpsOdomCallback, this, std::placeholders::_1));

        dvl_dead_reckon_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dvl_dead_reckon", 10,
            std::bind(&FactorStateNode::dvlDeadReckonCallback, this, std::placeholders::_1));

        // Set up the publisher
        factor_state_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/factor_state", 10);
    }

private:

    // gtsam stuff
    NonlinearFactorGraph graph;
    Values initialEstimate;
    ISAM2 isam;
    Values result;
    Vector6 std_noise;
    Vector6 sigmas;
    noiseModel::Diagonal::shared_ptr odometryNoise;



    void update_est() {
        isam.update(graph,initialEstimate);
        result = isam.calculateEstimate();
        graph.resize(0);
        initialEstimate.clear();
    }



    // Callback functions for each topic
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Process the IMU data here
        RCLCPP_INFO(this->get_logger(), "Received IMU data.");
        // Example: Print the IMU orientation
        RCLCPP_INFO(this->get_logger(), "Orientation x: %f, y: %f, z: %f, w: %f",
                    msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

        // Integrate data processing logic here
        publishFactorState();
    }

    void gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Process the GPS odometry data here
        RCLCPP_INFO(this->get_logger(), "Received GPS odometry data.");
        // Example: Print the position
        RCLCPP_INFO(this->get_logger(), "Position x: %f, y: %f",
                    msg->pose.pose.position.x, msg->pose.pose.position.y);

        // Integrate data processing logic here
        publishFactorState();
    }

    void dvlDeadReckonCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Process the DVL dead reckoning data here
        RCLCPP_INFO(this->get_logger(), "Received DVL dead reckoning data.");
        // Example: Print the velocity
        RCLCPP_INFO(this->get_logger(), "Position x: %f, y: %f",
                    msg->pose.pose.position.x, msg->pose.pose.position.y);

        // Integrate data processing logic here
        publishFactorState();
    }

    void publishFactorState()
    {
        // Create and publish the /factor_state message
        auto factor_state_msg = nav_msgs::msg::Odometry();
        // Populate the factor_state_msg here with the processed data
        factor_state_msg->pose.pose.position.x = 
        factor_state_msg->pose.pose.position.y = 
        
        factor_state_publisher_->publish(factor_state_msg);
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_dead_reckon_subscriber_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr factor_state_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FactorStateNode>());
    rclcpp::shutdown();
    return 0;
}
