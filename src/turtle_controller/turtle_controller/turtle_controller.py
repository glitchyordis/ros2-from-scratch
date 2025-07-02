#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose

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
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        
    def pose_callback(self, pose: Pose):
        cmd = Twist()
        if pose.x < 5.5:
            cmd.linear.x = 1.0
            cmd.angular.z = 1.0
        else:
            cmd.linear.x = 2.0
            cmd.angular.z = 2.0
        self.cmd_vel_pub_.publish(cmd)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()