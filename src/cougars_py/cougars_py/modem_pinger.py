
import rclpy
from rclpy.node import Node
from frost_interfaces.msg import ModemSend, ModemRec
from .seatrac_enums import CID_E, AMSGTYPE_E

CONFIG_FILE_PATH = "./seatrac_logger_config.toml"
TIMER_PERIOD_SECONDS = 0.01


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
        self.ping_delay = self.get_parameter("ping_delay_seconds")
        self.n_vehicles = self.get_parameter("number_of_vehicles")
        self.vehicle_order = self.get_parameter("vehicle_order")
        self.target_id = self.get_parameter("target_id")
        self.request_response = self.get_parameter("request_response")


    def send_ping(self):
        request = ModemSend()
        request.msg_id      = CID_E.CID_PING_SEND
        request.dest_id     = self.target_id
        request.msg_type    = AMSGTYPE_E.MSG_REQX
        self.modem_publisher_.publish(request)


def main(args=None):
    rclpy.init(args=args)
    seatrac_logger = SeatracPinger()
    rclpy.spin(seatrac_logger)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
