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
    //set serial to command line input if given
    // std::string serial_port;
    // if (argc == 1) { serial_port = "/dev/ttyUSB0"; }
    // else { serial_port = argv[1]; }


    std::string serial_port = "/dev/ttyUSB0"
    MyDriver seatrac(serial_port);

    calibration::calibrateAccelerometer(seatrac, std::cout, std::cin, false);
    calibration::calibrateMagnetometer(seatrac, std::cout, std::cin, false);
    //NOTE: This example calls calibrate with saveToEEPROM=false.
    //      That means the calibration settings will be lost once the 
    //      the seatrac beacon is powered off. Change saveToEEPROM to
    //      true to save the calibration settings perminantly.

    return 0;
}
