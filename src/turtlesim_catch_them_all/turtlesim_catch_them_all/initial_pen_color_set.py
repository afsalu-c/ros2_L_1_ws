#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen
 
 
class SetPenColorGreen(Node): 
    def __init__(self):
        super().__init__("set_pen_color_green") 
        self.set_pen_color_green_client_ = self.create_client(SetPen,"/turtle1/set_pen")

    def set_pen_color_service(self):
        while not self.set_pen_color_green_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for set pen service...")
        request = SetPen.Request()
        request.r = 0
        request.g = 255
        request.b = 0
        request.width = 2
        self.set_pen_color_green_client_.call_async(request)    

 
 
def main(args=None):
    rclpy.init(args=args)
    node = SetPenColorGreen()
    node.set_pen_color_service() 
    #rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()