#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <frost_interfaces/msg/modem_rec.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

class SeatracAHRSConverter : public rclcpp::Node
{
public:
    SeatracAHRSConverter()
        : Node("seatrac_ahrs_converter")
    {
        // https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?
        this->declare_parameter("magnetic_declination", 10.7); // 10.70° E for Utah Lake

        modem_subscriber_ = this->create_subscription<frost_interfaces::msg::ModemRec>(
            "modem_rec", 10, std::bind(&SeatracAHRSConverter::modem_callback, this, std::placeholders::_1));

        modem_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("modem_imu", 10);
    }

private:
    void modem_callback(const frost_interfaces::msg::ModemRec::SharedPtr msg)
    {
        if (msg->msg_id == 0x10) // 0x10 = CID_STATUS
        {
            auto modem_imu = std::make_shared<sensor_msgs::msg::Imu>();
            modem_imu->header.stamp = this->get_clock()->now();

            double yaw   = (M_PI/180.0)*(0.1 * msg->attitude_yaw - this->get_parameter("magnetic_declination").as_double());
            double pitch = (M_PI/180.0)*0.1 * msg->attitude_pitch;
            double roll  = (M_PI/180.0)*0.1 * msg->attitude_roll;

            // Create the NED quaternion using Eigen
            Eigen::Matrix3d R_ned(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

            // Define the NED to ENU conversion as a quaternion
            Eigen::Matrix3d R_ned_enu;
            R_ned_enu << 0, -1, 0,
                        -1, 0, 0,
                         0, 0, -1;
            //Eigen::Quaterniond q_ned_enu(R_ned_enu);

            // Convert to ENU coordinates by applying the NED-to-ENU rotation
            Eigen::Quaterniond q_enu(R_ned*R_ned_enu);

            modem_imu->orientation.x = q_enu.x();
            modem_imu->orientation.y = q_enu.y();
            modem_imu->orientation.z = q_enu.z();
            modem_imu->orientation.w = q_enu.w();
            modem_imu->orientation_covariance = {
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            };

            // Publish the IMU message
            modem_imu_pub_->publish(*modem_imu);
        }
    }

    rclcpp::Subscription<frost_interfaces::msg::ModemRec>::SharedPtr modem_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr modem_imu_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SeatracAHRSConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}