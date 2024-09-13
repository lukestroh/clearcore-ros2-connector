#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import socket
import json

SERVER_IP = "0.0.0.0"
SERVER_PORT = 8888

class UDPVelPublisher(Node):
    def __init__(self) -> None:
        super().__init__(node_name='udp_vel_publisher')
        self.pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=Float32,
            topic='linear_slider_vel',
            qos_profile=10
        )

        # Timer
        timer_period=0.000001
        self.timer: rclpy.timer.Rate = self.create_timer(timer_period_sec=timer_period, callback=self.timer_callback)

        # Socket server
        self.pub_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # DGRAM for UDP

        self.pub_socket.bind((SERVER_IP, SERVER_PORT))
        self.pub_socket.settimeout(0.0)
        return

    def timer_callback(self):
        msg = Float32()
        try:
            raw_data, addr = self.pub_socket.recvfrom(1024)
            self.get_logger().warn(f"{raw_data}")
            json_data: dict = json.loads(raw_data)

            status = json_data["status"]
            msg.data = float(json_data["servo_vel"])
            self.pub.publish(msg)
            # self.get_logger().info(f"Status: {status}, Velocity: {msg.data}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode error: {e}")
        except BlockingIOError as e:
            self.get_logger().debug("No data received")
        except KeyboardInterrupt:
            return
        except ValueError as e:
            print(f"{e}: Could not convert msg type to float.")
            status = -1
            msg.data = 0.0
        except KeyError as e:
            print(f"KeyError: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

        

        return
    
def main(args=None):
    rclpy.init(args=args)

    udp_publisher = UDPVelPublisher()

    rclpy.spin(udp_publisher)

    udp_publisher.pub_socket.close()

    return


if __name__ == "__main__":
    main()