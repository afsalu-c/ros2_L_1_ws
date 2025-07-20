#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from functools import partial
 
 
class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.service_client_ = self.create_client(SetLed,"set_led")
        self.get_logger().info("Battery node has been staeted.")

        self.create_timer(1.0,self.check_battery_state)
        self.battery_state_ = 'full'
        self.last_time_battery_state_changed_ = self.get_current_time_seconds()
        
    def get_current_time_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds  / 1e9  #or 1000000000.0
    
    def check_battery_state(self):
        time_now = self.get_current_time_seconds()
        if self.battery_state_ == 'full':
            if time_now - self.last_time_battery_state_changed_ > 4.0:
                self.battery_state_ = 'empty'
                self.get_logger().info("Battery is empty !, Charging.....")
                self.callback_battery_node(2,1)
                self.last_time_battery_state_changed_ = time_now
        
        else:
            if time_now - self.last_time_battery_state_changed_ > 6.0:
                self.battery_state_ = 'full'
                self.get_logger().info("Battery is now full.")
                self.callback_battery_node(2,0)
                self.last_time_battery_state_changed_ = time_now

    def callback_battery_node(self, led_number, state):
        while not self.service_client_.wait_for_service(1.0): #wait for 1 seconds
            self.get_logger().warn("waiting for set led service...") 
         
        request = SetLed.Request()
        request.led_number = led_number
        request.state = state

        future = self.service_client_.call_async(request)
        future.add_done_callback(partial(self.callback_set_led,request=request))
    
    def callback_set_led(self, future, request):
        response = future.result()
        if  response.success:
            self.get_logger().info('LED turned on')
        else:
            self.get_logger().info('LED not changed')
        #self.get_logger().info("led number = " + str(request.led_number) + " , " + ", state = " +
                                #str(request.state) + "Response = "  + str(response.success))       
 
 
def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()