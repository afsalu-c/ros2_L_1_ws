#!/usr/bin/env python3
import rclpy
from rclpy.node import Node #Node also a class
from std_msgs.msg import String #String is a class

class PublisherClass(Node):
    def __init__(self):
        super().__init__("publisher_node") #calling the method of the class 'Node' equivalent to Node("publisher_node")

        
        self.create_timer(2,self.timerCallback)
        self.i = 0
        

    def timerCallback(self):
        publisher = self.create_publisher(String,'/my_topic',10)
        msg = String()
        msg.data = "Hi, %d" % self.i
        publisher.publish(msg)
        self.get_logger().info('publishing : "%s"' % msg.data)
        self.i = self.i+1

def main(args = None):
    rclpy.init(args=args)
    node_v = PublisherClass()
    rclpy.spin(node_v)
    rclpy.shutdown()

if __name__ == "__main__": 
    main()