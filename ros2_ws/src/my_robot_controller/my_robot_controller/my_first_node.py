#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 

class MyNode(Node):

    def __init__(self):
        super().__init__("first_node") #initialize the node with Node from rclpy
        self.get_logger().info("Hello from Ros2") # setup logger
        self.create_timer(1.0,self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello")


def main(args=None):
    rclpy.init(args=args) #initialize ros2 communications

    node = MyNode() #run the node
    rclpy.spin(node) #to keep the node alive

    rclpy.shutdown() #Shutdown ros2 communications

if __name__ == '__main__':
    main()