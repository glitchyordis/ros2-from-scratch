#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from my_robot_interfaces.action import CountUntil


class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.count_until_server_ = ActionServer(
            self, # node to link action server to. topics and serices use `self.create...()`. here the object (self) is used instead.
            CountUntil, 
            "count_until", # action name
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback)
        
    # Every new received goal will be processed here first    
    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received a goal")
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting the goal, target number must be positive")
            return GoalResponse.REJECT
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
        
    # If a goal has been accepted, it will then be executed in this callback
    # After we are done with the goal execution we set a final state and return the result
    def execute_callback(self, goal_handle: ServerGoalHandle):
        target_number = goal_handle.request.target_number
        delay = goal_handle.request.delay
        result = CountUntil.Result()
        counter = 0

        self.get_logger().info("Executing the goal")
        for i in range (target_number):
            counter += 1
            self.get_logger().info(str(counter))
            time.sleep(delay)
            
        goal_handle.succeed() # we'l only use this for this simple example
        result.reached_number = counter
        return result
    
def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()