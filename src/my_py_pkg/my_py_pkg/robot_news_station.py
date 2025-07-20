#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

from rclpy.parameter import Parameter
 
 
class RobotNewsStationNode(Node): 
    def __init__(self):
        super().__init__("robot_news_station") 
        self.declare_parameter("robot_name","C3P0")
        self.robot_name_ = self.get_parameter("robot_name").value
        self.add_post_set_parameters_callback(self.parameters_callback)

        self.publisher_ = self.create_publisher(String, "robot_news", 10)
        self.timer_ = self.create_timer(1,self.publish_news)
        self.get_logger().info("Python robot news station has been started")

        #self.i = 0
        #self.robot_name_ = "C3PO"


    def publish_news(self):
        msg = String()
        msg.data = f"Hi, this is {self.robot_name_} from Robot News Station!" #data is the field 
        self.publisher_.publish(msg)

        #self.get_logger().info(msg.data)
        #self.i = self.i + 1

    def parameters_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == "robot_name":
                self.robot_name_ = param.value
    

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()