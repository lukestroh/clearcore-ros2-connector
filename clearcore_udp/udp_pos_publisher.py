#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import socket
import json

SERVER_HOST = "169.254.253.179"
SERVER_PORT = 8888

class UDPPosPublisher(Node):
    def __init__(self) -> None:
        super().__init__(node_name='udp_pos_publisher')
        self.pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=Float32,
            topic='linear_slider_pos',
            qos_profile=10
        )

        # Timer
        timer_period=0.000001
        self.timer: rclpy.timer.Rate = self.create_timer(timer_period_sec=timer_period, callback=self.timer_callback)

        # Socket server
        self.pub_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # DGRAM for UDP
        self.pub_socket.bind((SERVER_HOST, SERVER_PORT))

        return
    

    def timer_callback(self):
        msg = Float32()
        try:
            print("hello world")
            raw_data, addr = self.pub_socket.recvfrom(1024)
            print(raw_data)
            # raw_data = self.socketstreamer.readline().decode().rstrip()
            json_data: dict = json.loads(raw_data)
            msg.data = float(json_data["servo_velocity"])

        except ValueError as e:
            print(f"{e}: Could not convert msg type to float.")

        self.get_logger().info(f"Linear slider current velocity: {msg.data}")
        return
    


def main(args=None):
    rclpy.init(args=args)

    udp_publisher = UDPPosPublisher()

    rclpy.spin(udp_publisher)

    udp_publisher.pub_socket.close()

    return


if __name__ == "__main__":
    main()