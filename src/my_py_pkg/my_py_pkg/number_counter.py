#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool
 
 
class NumberCounterNode(Node): 
    def __init__(self):
        super().__init__("number_counter") 
        self.subscriber_ = self.create_subscription(Int64,"number",self.callback_number,10)
        self.counter_ = 0
        self.get_logger().info("Counter Reciver started")
        self.publisher_ = self.create_publisher(Int64,"number_count",10)
        self.server_ = self.create_service(
            SetBool,"reset_counter",self.callback_reset_counter)


    def callback_number(self, msg:Int64):
        self.counter_ += 1
        self.get_logger().info(f"Number of messages recived = {self.counter_}")
        msg2 = Int64()
        msg2.data = self.counter_
        self.publisher_.publish(msg2)

    def callback_reset_counter(self, request: SetBool.Request, response: SetBool.Response):
        if request.data == True:
            self.counter_ = 0
            response.message = "number_counter SET to zero"
            response.success = True
        else:
            response.message = "number_counter NOT SET to zero"
            response.success = False
        return response
          
def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()