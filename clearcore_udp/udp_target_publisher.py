#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import socket


CLEARCORE_IP = '169.254.57.177'
CLEARCORE_PORT = 8888
LOCAL_IP = "169.254.57.209"
LOCAL_PORT = 44644


class UDPTargetPublisher(Node):
    def __init__(self) -> None:
        super().__init__(node_name='udp_target_publisher') # init node with the node name
        self.pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=Float32,
            topic='linear_slider_target',
            qos_profile=10 # qos_profile or history depth
        )
        # Timer
        timer_period = 0.05 
        self.timer: rclpy.timer.Rate = self.create_timer(timer_period_sec=timer_period, callback=self.timer_callback)

        # Socket client
        self.pub_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.pub_socket.bind((LOCAL_IP, LOCAL_PORT))

        self.clearcore_addr = (CLEARCORE_IP, CLEARCORE_PORT)

        self.target = 0.0
        self.going_up: bool = True
        return
    
    def timer_callback(self):
        # Construct ROS msg, publish
        msg = Float32()

        
        if self.target == 30.0:
            self.going_up = False
        elif self.target == -30.0:
            self.going_up = True

        if self.going_up:
            self.target += 1.0
        else:
            self.target -= 1.0
        
        msg.data = self.target
        self.pub.publish(msg)

        status = 0
        
        # send data to ClearCoreCLEARCORE_HOST
        self.pub_socket.sendto(f"{status},{msg.data}".encode(), self.clearcore_addr)
        # log the info
        self.get_logger().info(f"Sent: {msg.data}")

        return
    



def main(args=None):
    rclpy.init(args=args)

    udp_publisher = UDPTargetPublisher()

    rclpy.spin(udp_publisher)

    udp_publisher.pub_socket.close()


if __name__ == "__main__":
    main()