#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args = None):
    rclpy.init(args=args)
    first_node_v = Node("first_node")  #"first_node_v" is the variable name and "first_node" is the node name
    second_node_v = Node("second_node")
    third_node_v = Node("third_node")
    first_node_v.get_logger().info("Hello I'm first node")

    msg = String() #created new msg string object
    msg.data = "Publishing from second_node"
    publisher_of_second_node = second_node_v.create_publisher(String,'/example_topic',10) ## Creates a publisher on the topic "topic" with a queue size of 10 messages

    rclpy.spin_once(third_node_v,timeout_sec=0.5) #BETTER adding small delay to ensure subscription node able to catch from publisher, but for me without that is also worked
    def callback(msg):
            third_node_v.get_logger().info(f'I heared : "{msg.data}"')
    

    subsription = third_node_v.create_subscription(String,'/example_topic',callback,10)
    publisher_of_second_node.publish(msg) #publishing msg


    try:
          while rclpy.ok():
                rclpy.spin_once(second_node_v,timeout_sec=0.1)
                rclpy.spin_once(third_node_v,timeout_sec=0.1)
 
    except KeyboardInterrupt:
          third_node_v.get_logger().info("Node stopped by the user")
    finally:
        rclpy.shutdown()
if __name__ =="__main__":
      main()