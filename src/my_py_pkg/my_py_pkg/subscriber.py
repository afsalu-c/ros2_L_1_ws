#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubsciberClass(Node):
    def __init__(self):
        super().__init__("subscriber_node")
        subsription = self.create_subscription(String,"/my_topic",self.listenerCallBack,10)
        
    def listenerCallBack(self, msg):
       self.get_logger().info(f"I heared : '{msg.data}'")


def main(args=None):
    rclpy.init(args=args)
    node = SubsciberClass()
    rclpy.spin(node)
    rclpy.shutdown()