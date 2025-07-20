#!/usr/bin/env python3
import math
from functools import partial
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("catch_closet_turtle_first", True)
        self.catch_closet_turtle_first = self.get_parameter(
            "catch_closet_turtle_first").value
        self.turtle_to_catch_: Turtle = None
        self.flag_ = "green"
        self.pose_: Pose = None
        self.cmd_vel_pub_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub_ = self.create_subscription(
            Pose, '/turtle1/pose', self.callback_pose, 10)
        self.alive_turtles_sub_ = self.create_subscription(
            TurtleArray,"alive_turtles",self.callback_alive_turtles,10)
        self.catch_turtle_client_ = self.create_client(
            CatchTurtle,"cath_turtle")
        self.control_loop_timer_  = self.create_timer(
            0.01,self.control_loop)
        self.set_pen_color_client_ = self.create_client(SetPen,"/turtle1/set_pen")
    def callback_pose(self, pose: Pose):
        self.pose_ = pose

    def callback_alive_turtles(self, msg: TurtleArray):
        if len(msg.turtles) > 0: 
            closet_turtle = None
            closet_turtle_distance = None

            for turtle in msg.turtles:
                dist_x = turtle.x - self.pose_.x
                dist_y = turtle.y - self.pose_.y
                distance = math.sqrt(dist_x ** 2 + dist_y ** 2)
                if closet_turtle == None or distance < closet_turtle_distance:
                    closet_turtle = turtle
                    closet_turtle_distance = distance
                self.turtle_to_catch_ = closet_turtle
        else:
            #self.turtle_to_catch_ = msg.turtles[0]
            self.turtle_to_catch_ = None
    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x **2 + dist_y ** 2)

        cmd = Twist()
        kp_linear = 2.0
        kp_angular = 6.0

        if distance  > 0.5: #position
            cmd.linear.x = kp_linear * distance

            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            cmd.angular.z = 0.0
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            cmd.angular.z = kp_angular * diff
        else: #target reached
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle_service(self.turtle_to_catch_.name, self.flag_)
            self.turtle_to_catch_ = None
    

        # if self.flag_ == "green":
        #     self.set_pen_color_service("red")
        #     self.flag_ = "red"
        #     self.cmd_vel_pub_.publish(cmd)
        # else:
        #     self.set_pen_color_service("green")
        #     self.flag_ = "green"
        #     self.cmd_vel_pub_.publish(cmd)

        self.cmd_vel_pub_.publish(cmd)

                   
    def call_catch_turtle_service(self, turtle_name, flag):
        while not self.catch_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for kill service...")
        request = CatchTurtle.Request()
        request.name = turtle_name

        future = self.catch_turtle_client_.call_async(request)
        self.set_pen_color_service(flag)
        future.add_done_callback(partial(self.callback_call_catch_turtle_service, turtle_name=turtle_name))
    
    def callback_call_catch_turtle_service(self, future, turtle_name):
        response: CatchTurtle.Response = future.result()
        if not response.success:
            self.get_logger().error("Turtle " + turtle_name + " could not be removes")

    def set_pen_color_service(self, flag):
        while not self.set_pen_color_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for set pen service...")
        request = SetPen.Request()
        if flag == "green":
            request.r = 255
            request.g = 0
            request.b = 0
            request.width = 2
            self.flag_ = "red"
        else:
            request.r = 0
            request.g = 255
            request.b = 0
            request.width = 2
            self.flag_ = "green"
        self.set_pen_color_client_.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
