
.. _program_listing_file_src_coug_kinematics.cpp:

Program Listing for File coug_kinematics.cpp
============================================

|exhale_lsh| :ref:`Return to documentation for file <file_src_coug_kinematics.cpp>` (``src/coug_kinematics.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include <chrono>
   #include <functional>
   #include <memory>
   #include <string>
   
   #include "frost_interfaces/msg/u_command.hpp"
   #include "rclcpp/rclcpp.hpp"
   
   using namespace std::chrono_literals;
   using std::placeholders::_1;
   
   rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
   auto qos = rclcpp::QoS(
       rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
       qos_profile);
   
   class CougKinematics : public rclcpp::Node {
   public:
     CougKinematics() : Node("coug_kinematics") {
   
       this->declare_parameter("trim_ratio", 0.0);
   
       this->declare_parameter("top_fin_offset", 0.0);
   
       this->declare_parameter("right_fin_offset", 0.0);
   
       this->declare_parameter("left_fin_offset", 0.0);
   
       command_publisher_ =
           this->create_publisher<frost_interfaces::msg::UCommand>(
               "kinematics/command", 10);
   
       command_subscription_ =
           this->create_subscription<frost_interfaces::msg::UCommand>(
               "controls/command", 10,
               std::bind(&CougKinematics::command_callback, this, _1));
     }
   
   private:
     void command_callback(const frost_interfaces::msg::UCommand &msg) {
   
       auto command = frost_interfaces::msg::UCommand();
       command.header.stamp = msg.header.stamp;
       command.fin[0] =
           msg.fin[0] + this->get_parameter("top_fin_offset").as_double() +
           this->get_parameter("trim_ratio").as_double() * msg.thruster;
       command.fin[1] =
           msg.fin[1] + this->get_parameter("right_fin_offset").as_double() +
           this->get_parameter("trim_ratio").as_double() * msg.thruster;
       command.fin[2] =
           msg.fin[2] + this->get_parameter("left_fin_offset").as_double() +
           this->get_parameter("trim_ratio").as_double() * msg.thruster;
       command.thruster = msg.thruster;
   
       command_publisher_->publish(command);
     }
   
     // micro-ROS objects
     rclcpp::Publisher<frost_interfaces::msg::UCommand>::SharedPtr
         command_publisher_;
     rclcpp::Subscription<frost_interfaces::msg::UCommand>::SharedPtr
         command_subscription_;
   };
   
   int main(int argc, char *argv[]) {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<CougKinematics>());
     rclcpp::shutdown();
     return 0;
   }
