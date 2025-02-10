#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <seatrac_interfaces/msg/modem_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#define SQRT_OF_2_OVER_2 0.70710678118


class SeatracAHRSConverter : public rclcpp::Node {
public:
  SeatracAHRSConverter() : Node("seatrac_ahrs_converter") {
    // https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?
    this->declare_parameter("magnetic_declination",
                            10.7); // 10.70° E for Utah Lake
    magnetic_declination = this->get_parameter("magnetic_declination").as_double();

    modem_subscriber_ =
        this->create_subscription<seatrac_interfaces::msg::ModemStatus>(
            "modem_status", 10,
            std::bind(&SeatracAHRSConverter::modem_callback, this,
                      std::placeholders::_1));

    modem_imu_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>("modem_imu", 10);

  }

private:
  void modem_callback(const seatrac_interfaces::msg::ModemStatus::SharedPtr msg) {
    auto modem_imu = std::make_shared<sensor_msgs::msg::Imu>();
    modem_imu->header.stamp = msg->header.stamp;

    double yaw   = (M_PI / 180.0) * (0.1 * msg->attitude_yaw + magnetic_declination);
    double pitch = (M_PI / 180.0) * 0.1 * msg->attitude_pitch;
    double roll  = (M_PI / 180.0) * 0.1 * msg->attitude_roll;

    // Create the NED quaternion using Eigen
    Eigen::Matrix3d R_ned(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    // Define the NED to ENU conversion as a quaternion
    Eigen::Matrix3d R_ned_enu;
    R_ned_enu << 
        0, -1,  0, 
       -1,  0,  0, 
        0,  0, -1;

    // Convert to ENU coordinates by applying the NED-to-ENU rotation
    Eigen::Quaterniond q_enu(R_ned * R_ned_enu);

    modem_imu->orientation.x = q_enu.x();
    modem_imu->orientation.y = q_enu.y();
    modem_imu->orientation.z = q_enu.z();
    modem_imu->orientation.w = q_enu.w();
    // modem_imu->orientation_covariance = {
    //     1.0, 0.0, 0.0,
    //     0.0, 1.0, 0.0, 
    //     0.0, 0.0, 1.0
    // };



    // The gyroscope is mounted with a 45 degree offset from the modem frame
    // So a linear transformation is needed. Determined experimentally.
    modem_imu->angular_velocity.x = SQRT_OF_2_OVER_2*(msg->gyro_y - msg->gyro_x);
    modem_imu->angular_velocity.y = SQRT_OF_2_OVER_2*(msg->gyro_x + msg->gyro_y);
    modem_imu->angular_velocity.z = -msg->gyro_z;

    // RCLCPP_INFO(this->get_logger(), "\t%d\t%d\t%d", 
    //   (int)(modem_imu->angular_velocity.x/250), 
    //   (int)(modem_imu->angular_velocity.y/250),
    //   (int)(modem_imu->angular_velocity.z/250)
    // );

    // The accelerometer is mounted with a 45 degree offset from the modem frame
    // so a linear transformation is need. Determined experimentally.
    modem_imu->linear_acceleration.x =  SQRT_OF_2_OVER_2*(msg->acc_x - msg->acc_y);
    modem_imu->linear_acceleration.y = -SQRT_OF_2_OVER_2*(msg->acc_x + msg->acc_y);
    modem_imu->linear_acceleration.z =  msg->acc_z;

    // RCLCPP_INFO(this->get_logger(), "\t%d\t%d\t%d", 
    //   (int)(modem_imu->linear_acceleration.x/25), 
    //   (int)(modem_imu->linear_acceleration.y/25),
    //   (int)(modem_imu->linear_acceleration.z/25)
    // );

    // Publish the IMU message
    modem_imu_pub_->publish(*modem_imu);

  }

  rclcpp::Subscription<seatrac_interfaces::msg::ModemStatus>::SharedPtr
      modem_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr modem_imu_pub_;
  double magnetic_declination;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SeatracAHRSConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}