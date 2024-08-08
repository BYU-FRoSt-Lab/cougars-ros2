#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/TwistWithCovarianceStamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;



class DVLSender : public rclcpp::Node
{
  public:
    DVLSender():Node("dvl_data_sender")
    {
      publisher_dvl_velocity = this->create_publisher<std_msgs::msg::String>("dvl_velocity", 10);
      subscriber_dvl_data = this->create_subscription<std_msgs::msg::String>("/dvl/data", 10, std::bind(&DVLSender::dvl_data_callback, this, _1));

    }

  private:
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_dvl_velocity;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_dvl_all;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DVLSender>());
  rclcpp::shutdown();
  return 0;
}