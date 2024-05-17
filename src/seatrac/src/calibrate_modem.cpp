#include <iostream>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>
#include <seatrac_driver/commands.h>
#include <seatrac_driver/Calibration.h>

using namespace std::chrono_literals;
using namespace narval::seatrac;

// The class needs to inherit from both the ROS node and driver classes
class MyDriver : public SeatracDriver
{
    public:

    MyDriver(const std::string& serialPort = "/dev/ttyUSB0") :
        SeatracDriver(serialPort)
    {}

    // this method is called on any message returned by the beacon.
    void on_message(CID_E msgId, const std::vector<uint8_t>& data) {
        switch(msgId) {
            default:
                std::cout << "Got message : " << msgId << std::endl << std::flush;
                break;

            case CID_STATUS: {
                messages::Status status;
                status = data;
                calibration::printCalFeedback(std::cout, status);
            } break;
        }
    }
};

int main(int argc, char *argv[])
{
    std::string serial_port = "/dev/ttyUSB0";
    MyDriver seatrac(serial_port);
    calibration::turnOffCalFeedback(seatrac);

    int action;
    std::cout << "Running Seatrac Modem Calibration:" << std::endl
              << "Which calibration procedure would you like to execute?" << std::endl
              << "\t1) Magnetometer Calibration" << std::endl
              << "\t2) Accelerometer Calibration" << std::endl
              << "\t3) Both Magnetometer and Acceleromter Calibration" << std::endl
              << "\t4) Dry run - cal settings only saved to RAM, not EEPROM" << std::endl
              << "\t5 or more) Exit" << std::endl
              << "Enter a number: ";
    scanf("%d", &action);
    switch(action) {
        case 1: {
            calibration::calibrateMagnetometer(seatrac, std::cout, std::cin, true);
        } break;
        case 2: {
            calibration::calibrateAccelerometer(seatrac, std::cout, std::cin, true);
        } break;
        case 3: {
            calibration::calibrateMagnetometer(seatrac, std::cout, std::cin, true);
            calibration::calibrateAccelerometer(seatrac, std::cout, std::cin, true);
        } break;
        case 4: {
            calibration::calibrateMagnetometer(seatrac, std::cout, std::cin, false);
            calibration::calibrateAccelerometer(seatrac, std::cout, std::cin, false);
        } break;
        default: {
            std::cout << "Exiting Seatrac Modem Calibration" << std::endl;
        } break;
    }

    return 0;
}
