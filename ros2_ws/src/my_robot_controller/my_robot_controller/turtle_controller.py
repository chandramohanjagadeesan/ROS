#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.previous_x = 0
        self.get_logger().info("Turtle Controller is Started")
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)

    def pose_callback(self,pose:Pose):
        cmd = Twist()
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

        if pose.x > 5.5 and self.previous_x <=5.5: # Services needs to be called only when needed
            self.get_logger().info("Setpen color Red!")
            self.previous_x = pose.x
            self.call_set_pen_service(255, 0, 0, 3, 0)
        elif pose.x <= 5.5 and self.previous_x > 5.5:
            self.get_logger().info("Setpen color Green!")
            self.previous_x = pose.x
            self.call_set_pen_service(0, 255, 0, 3, 0)

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(
            SetPen, "/turtle1/set_pen") #create a cleint service 
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service") #wait for the service
        
        request = SetPen.Request() #create a request
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request) #send the request to the serer gives a future object
        future.add_done_callback(partial(self.callback_set_pen)) #adding callback when the service replies

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("service call failed: %r" % (e,))  #add exceptions if callback does not work

def main(args=None):
    rclpy.init(args=args)
    node  = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()