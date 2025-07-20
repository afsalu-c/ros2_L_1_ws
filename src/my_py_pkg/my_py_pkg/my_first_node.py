#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):#inherit Node class
    def __init__(self): #constructor 
        super().__init__("py_test")
        self.counter_ = 0
        self.get_logger().info("Hello World aaaaaaaaaaaaaaaaaaaa")
        self.create_timer(0.5,self.timerCallback)#every 0.5 second hust give the mame of function, not call function

    def timerCallback(self):
        self.get_logger().info("Hi" + " " + str(self.counter_))
        self.counter_ += 1
       

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__": 
    main()
