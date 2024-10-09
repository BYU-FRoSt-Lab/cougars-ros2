#include <rclcpp/rclcpp.hpp>
#include "seatrac_interfaces/msg/modem_send.hpp"
#include <seatrac_driver/SeatracEnums.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using seatrac_interfaces::msg::ModemSend;
using namespace narval::seatrac;

class SeatracPinger : public rclcpp::Node
{
public:
    SeatracPinger() : Node("modem_pinger")
    {
        this->declare_parameter<int>("ping_delay_seconds", 1);
        this->declare_parameter<int>("number_of_vehicles", 1);
        this->declare_parameter<int>("vehicle_order", 1);
        this->declare_parameter<int>("target_id", 0);
        this->declare_parameter<bool>("request_response", false);

        this->ping_delay_ = this->get_parameter("ping_delay_seconds").as_int();
        this->n_vehicles_ = this->get_parameter("number_of_vehicles").as_int();
        this->vehicle_order_ = this->get_parameter("vehicle_order").as_int();
        this->target_id_ = this->get_parameter("target_id").as_int();
        this->request_response_ = this->get_parameter("request_response").as_bool();

        modem_publisher_ = this->create_publisher<ModemSend>("modem_send", 10);

        std::thread(&SeatracPinger::repeat_call_ping, this).detach();
    }

private:
    int ping_delay_;
    int n_vehicles_;
    int vehicle_order_;
    int target_id_;
    bool request_response_;
    rclcpp::Publisher<ModemSend>::SharedPtr modem_publisher_;

    void repeat_call_ping()
    {
        int total_seconds_per_round = ping_delay_ * n_vehicles_;
        int my_ping_second = ping_delay_ * (vehicle_order_ - 1);

        while (rclcpp::ok())
        {
            auto now = std::chrono::system_clock::now();
            auto time_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
            int seconds_since_midnight = time_since_epoch.count() % (24 * 60 * 60);
            int seconds_in_round = seconds_since_midnight % total_seconds_per_round;
            int sleep_time = (my_ping_second - seconds_in_round + total_seconds_per_round) % total_seconds_per_round;

            std::this_thread::sleep_for(std::chrono::seconds(sleep_time));

            send_ping();
            std::this_thread::sleep_for(2s);
        }
    }

    void send_ping()
    {
        auto request = ModemSend();
        request.msg_id = CID_E::CID_DAT_SEND;
        request.dest_id = target_id_;
        request.msg_type = request_response_ ? AMSGTYPE_E::MSG_REQU : AMSGTYPE_E::MSG_OWAYU;
        modem_publisher_->publish(request);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SeatracPinger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}