#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedPanelState
from my_robot_interfaces.srv import SetLed
 
 
class LedPanelNode(Node): 
    def __init__(self):
        super().__init__("led_panel") 
        self.led_state_pub = self.create_publisher(LedPanelState, 'led_panel_state',10)
        self.timer_ = self.create_timer(5.0,self.pub_panel_state)
        self.server_ = self.create_service(SetLed,"set_led",self.callback_set_led)

        self.led_state_= [0, 0, 0]
        self.declare_parameter("led_state",[0, 0, 0])
        self.led_state_ = self.get_parameter("led_state").value
        

        self.get_logger().info("Led panel node has been started.")

    def pub_panel_state(self):
        msg = LedPanelState()
        msg.led_states = self.led_state_
        self.led_state_pub.publish(msg)
    
    def callback_set_led(self, request: SetLed.Request, response: SetLed.Response):
        LED_number = request.led_number
        LED_state = request.state

        
        if LED_number < 0 or LED_number > 2:
            response.success = False
            return response
        
        if LED_state not in range(0, 2):
            response.success = False    
            return response           
            

        self.led_state_[LED_number] = LED_state
        self.pub_panel_state() # WE CAN PPUBLISH FROM ANY WHERE, by calling function immediately after updating
        response.success = True
        return response



def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()