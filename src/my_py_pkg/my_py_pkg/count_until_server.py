#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import CountUntil

# when using  multi-threaded executors, we also need to use callback groups.
# Here, ReentrantCallbackGroup will allow all callbacks to be executed in parallel. This means that you can have several goal, cancel,
# and execute callbacks running at the same time for one action server.

class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.count_until_server_ = ActionServer(
            self, # node to link action server to. topics and serices use `self.create...()`. here the object (self) is used instead.
            CountUntil, 
            "count_until", # action name
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action server has been started.")

        
    # Every new received goal will be processed here first    
    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Received a goal")
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting the goal, target number must be positive")
            return GoalResponse.REJECT
        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT
        
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT # to reject use .REJECT
        
    # If a goal has been accepted, it will then be executed in this callback
    # After we are done with the goal execution we set a final state and return the result
    # When executing the goal we also check if we need to cancel it
    #  you need to set the goal’s final state and return a result, even if you cancel the goal. 
    def execute_callback(self, goal_handle: ServerGoalHandle):
        target_number = goal_handle.request.target_number
        delay = goal_handle.request.delay
        result = CountUntil.Result()
        feedback = CountUntil.Feedback()
        counter = 0

        self.get_logger().info("Executing the goal")
        for i in range (target_number):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling goal")
                goal_handle.canceled()
                result.reached_number = counter
                return result
            
            counter += 1
            self.get_logger().info(str(counter))
            
            # fill in diff field on feedback obj and send to client
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(delay)
            
        goal_handle.succeed() # we'l only use this for this simple example
        result.reached_number = counter
        return result
    
def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()