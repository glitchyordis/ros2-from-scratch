#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.client import Client
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from my_robot_interfaces.srv import ActivateTurtle

"""
To see what interface we need, run 
ros2  run turtlesim turtlesim_node
ros2 topic list

# the two below will tell you msg types and their fields
ros2 topic info /turtle1/cmd_vel
    Type: geometry_msgs/msg/Twist
    Publisher count: 0
    Subscription count: 1
ros2 topic info /turtle1/pose
    Type: turtlesim/msg/Pose
    Publisher count: 1
    Subscription count: 0
    
To test this you need:
ros2 run turtlesim turtlesim_node
ros2 run turtle_controller turtle_controller
"""

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.previous_x_ = 0.0
        self.is_active_ = True
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self.set_pen_client_ = self.create_client(SetPen, "/turtle1/set_pen")
        self.activate_turtle_service_ = self.create_service(ActivateTurtle, "activate_turtle", self.callback_activate_turtle)
        
    def pose_callback(self, pose: Pose):
        if self.is_active_:
            cmd = Twist()
            if pose.x < 5.5:
                cmd.linear.x = 1.0
                cmd.angular.z = 1.0
            else:
                cmd.linear.x = 2.0
                cmd.angular.z = 2.0
            self.cmd_vel_pub_.publish(cmd)
            
            """
            We could send a service request every time, even if the color would be the same as the previous one. Why “optimize” the code?
            
            The reason is that the callback_pose() method will be called a lot.
            $ ros2 topic hz /turtle1/pose
            average rate: 62.515
            
            This is not really a problem. We also publish on the /turtle1/cmd_vel topic at 62 Hz. Again, that’s not a problem. Publishers and subscribers can sustain a high frequency (with a bigger message size, this could become complicated, but here, the messages are really small).
            
            what if we send a request to a service 62 times per second? 
            Services are not made for high-frequency requests, and this could seriously affect the performance of the application. Also, if you find yourself having to call a service at 62 Hz, then you probably have a design problem, and you either need to modify your code to reduce the frequency or use a publish/subscribe mechanism instead.
            
            So, what we do in the code is make sure we only call the service when it’s needed—that is, when the turtle switches from one side to the other.
            """
            if pose.x > 5.5 and self.previous_x_ <= 5.5:
                self.previous_x_ = pose.x
                self.get_logger().info("Set color to red.")
                self.call_set_pen(255, 0, 0)
            elif pose.x <= 5.5 and self.previous_x_ > 5.5:
                self.previous_x_ = pose.x
                self.get_logger().info("Set color to green.")
                self.call_set_pen(0, 255, 0)
        
    def call_set_pen(self, r, g, b):
        while not self.set_pen_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        future = self.set_pen_client_.call_async(request)
        future.add_done_callback(self.callback_set_pen_response)
        
    def callback_set_pen_response(self, future):
        # we don’t check what’s inside the response since the response is empty (it exists but it doesn’t contain a field).
        self.get_logger().info("Successfully changed pen color")
    
    def callback_activate_turtle(self, request: ActivateTurtle.Request, response: ActivateTurtle.Response):
        self.is_active_ = request.activate
        if request.activate:
            response.message = "Starting the turtle"
        else:
            response.message = "Stopping the turtle"
        return response
    
    
    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()