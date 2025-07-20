#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
 

def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client_no_oop") 
    
    client = node.create_client(AddTwoInts,"add_two_ints") 
    ''' 
    may show error while rinning if server not running 
    '''
    while not client.wait_for_service(1.0): #wait for 1 seconds
        node.get_logger().warn("waiting for Add Two Ints server...")
    
    request = AddTwoInts.Request()
    request.a = 9
    request.b = 56

    future = client.call_async(request) #treturn a obk=ject in future
    rclpy.spin_until_future_complete(node, future) #run untill future is complete

    response = future.result()
    node.get_logger().info(str(request.a) + " + " + 
                              str(request.b) + " = " + str(response.sum))

    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()