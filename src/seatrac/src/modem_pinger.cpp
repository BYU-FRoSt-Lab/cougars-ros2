#include <rclcpp/rclcpp.hpp>
#include "seatrac_interfaces/msg/modem_send.hpp"
#include <seatrac_driver/SeatracEnums.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using seatrac_interfaces::msg::ModemSend;
using namespace narval::seatrac;



/**
 * @brief Sends acoustic pings on a schedule
 * @author Clayton Smith
 * @date September 2024
 * 
 * Broadcasts acoustic pings over the seatrac x150 beacon on a timed 
 * schedule to avoid message overlaps in a fleet of UUVs. Signals are 
 * sent in a round-robin style, where each vehicle takes a turn sending a 
 * ping. The schedule is managed by the rasberry pi clock in each 
 * vehicle, so that it does not depend on every signal being receieved.
 * 
 * The ping schedule is based on 3 parameters: ping_delay_seconds, number_of_vehicles, 
 * and vehicle_order, and uses the following formula:
 * - ping_cycle_length = ping_delay_seconds * number_of_vehicles
 * - seconds_into_cycle = seconds_since_midnight % ping_cycle_length
 * - time_until_ping = (ping_delay_seconds * vehicle_order - seconds_into_cycle) % ping_cycle_length
 * 
 * @warning raspberry pi clocks should be synchonized between
 * vehicles before using this node.
 * 
 * 
 * Publishes:
 * - modem_send (seatrac_interfaces/msg/ModemSend)
 */
class SeatracPinger : public rclcpp::Node
{
public:
    SeatracPinger() : Node("modem_pinger")
    {
        /**
         * @param ping_delay_seconds
         * 
         * The time between a ping from any vehicle in the water. For example
         * if Coug 1, Coug 2, and Coug 3 are pinging each other in that order,
         * then this would be the time between pings from Coug 1 and Coug 2.
         * WARNING: ping_delay_seconds should be the same for all vehicles.
         */
        this->declare_parameter<int>("ping_delay_seconds", 5);

        /**
         * @param number_of_vehicles
         * 
         * The number of vehicles sending pings. The ping cycle will reset after 
         * every vehicle has sent a ping. The length of time for a complete ping
         * cycle, or the time between pings from this vehicle is ping_delay_seconds
         * multiplied by number_of_vehicles.
         * WARNING: number_of_vehicles should be the same for all vehicles.
         */
        this->declare_parameter<int>("number_of_vehicles", 1);

        /**
         * @param vehicle_ID
         *
         * The Coug-UV vehicle ID, used to set the beacon ID.
         * Also used to set the vehicle_order if vehicle_order is undefined.
         */
        this->declare_parameter<int>("vehicle_ID", 1);
        /**
         * @param vehicle_order
         * 
         * If specified, the position of this vehicle in the ping order. 
         * By default this will be the vehicle_ID, so coug1
         * will ping first, then coug2, etc.
         * 
         * WARNING: If specified, vehicle_order should different for all vehicles.
         * If two vehicles have the same vehicle_order, they will send pings 
         * at the same time.
         */
        this->declare_parameter<int>("vehicle_order", -1); //-1 indicates unspecified

        /**
         * @param request_response
         * 
         * A boolean indicating whether or not to request a response signal from
         * the vehicle you're addressing this ping to. Response signals can be used
         * to calculate range between vehicles using 2-way-time-of-flight. 
         * 
         * WARNING: If request_response is true, the target_id must not be zero 
         * (between 1 and 16), indicating that this acoustic message is addressed 
         * to a specific vehicle and not all vehicles.
         */
        this->declare_parameter<bool>("request_response", false);

        /**
         * @param target_id
         * 
         * The vehicle to address this ping to. If target_id is zero then the ping sent
         * will be addressed to all vehicles. If the target_id is between 1 and 16, it
         * will address the message to the vehicle with vehicle_ID=target_id. 
         * 
         * Acoustic messages are always broadcasted to and received by every vehicle,
         * regardless of which specific vehicle the message is addressed to. However, only
         * the vehicle this message is addressed to will respond with a return message.
         * And so, if request_response is true, this parameter must be set between 1 and 16.
         * If request_response is false, it makes no difference what the target_id is, so you
         * leave it at 0 to send to all beacons.
         */
        this->declare_parameter<int>("target_id", 0);

        this->ping_delay_ = this->get_parameter("ping_delay_seconds").as_int();
        this->n_vehicles_ = this->get_parameter("number_of_vehicles").as_int();
        this->vehicle_order_ = this->get_parameter("vehicle_order").as_int();
        if(this->vehicle_order_ == -1) this->vehicle_order_ = this->get_parameter("vehicle_ID").as_int();
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