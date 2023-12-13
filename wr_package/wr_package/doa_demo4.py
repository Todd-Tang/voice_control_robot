import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import fftconvolve

import pyroomacoustics as pra
from pyroomacoustics.doa import circ_dist

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        
        super()._init_('node_name')
        self.create_timer(1, self.timer_callback)
        
    def timer_callback(self):
        # print a ROS2 log on the terminal with a great message!
        self.get_logger().info("Moe yo Byakugan! Kore ga watashi no nindō yo")

def main(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = MyNode()
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclpy.spin(node)
    # shutdown the ROS2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
