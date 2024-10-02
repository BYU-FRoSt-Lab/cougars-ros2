import rclpy
from rclpy.node import Node
from frost_interfaces.msg import ModemSend
from .seatrac_enums import CID_E, AMSGTYPE_E
import time
from datetime import datetime, timedelta
import threading

# TODO: Convert to C++, add to seatrac package

class SeatracPinger(Node):

    def __init__(self):
        super().__init__('modem_pinger')

        self.declare_parameters(
            namespace='',
            parameters=[
                ("ping_delay_seconds", rclpy.Parameter.Type.INTEGER),
                ("number_of_vehicles", rclpy.Parameter.Type.INTEGER),
                ("vehicle_order", rclpy.Parameter.Type.INTEGER),
                ("target_id", rclpy.Parameter.Type.INTEGER),
                ("request_response", rclpy.Parameter.Type.BOOL)
            ]
        )
        self.ping_delay = self.get_parameter("ping_delay_seconds").get_parameter_value().integer_value # The time between pings sent from any beacon
        self.n_vehicles = self.get_parameter("number_of_vehicles").get_parameter_value().integer_value # The number of vehicles sending pings
        self.vehicle_order = self.get_parameter("vehicle_order").get_parameter_value().integer_value   # what place this beacon is in the order of all the beacons
        self.target_id = self.get_parameter("target_id").get_parameter_value().integer_value           # the id of the target beacon. If 0, it targets all beacons.
        self.request_response = self.get_parameter("request_response").get_parameter_value().bool_value

        self.modem_publisher_  = self.create_publisher(ModemSend, 'modem_send', 10)

        thread = threading.Thread(target=self.repeat_call_ping())
        thread.start()




    def repeat_call_ping(self):
        # each hour is epoch
        total_seconds_per_round = self.ping_delay * self.n_vehicles
        my_ping_second = self.ping_delay * (self.vehicle_order -1)

        while True:
            now = datetime.now()
            seconds_since_midnight = now.hour*24*60 + now.minute*60 + now.second
            seconds_in_round = seconds_since_midnight % total_seconds_per_round
            sleep_time = (my_ping_second - seconds_in_round)%total_seconds_per_round
            time.sleep(sleep_time)

            self.send_ping()
            time.sleep(2)



    # This is where you tell the modem what to send
    def send_ping(self):
        request = ModemSend()
        request.msg_id      = CID_E.CID_DAT_SEND    # DATA protocol supports one way and request/response messages, so using without packet data.
        request.dest_id     = self.target_id        # The vehicle (and beacon) id to send the message too (0 for all beacons)
        if(self.request_response):
            request.msg_type = AMSGTYPE_E.MSG_REQU  # requests a response with usbl information
        else:
            request.msg_type = AMSGTYPE_E.MSG_OWAYU # sends a one way message with usbl information
        self.modem_publisher_.publish(request)


def main(args=None):
    rclpy.init(args=args)
    seatrac_logger = SeatracPinger()
    rclpy.spin(seatrac_logger)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
