#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.node import Node

from my_robot_interfaces.action import CountUntil


class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self.count_until_client_ = ActionClient(self, CountUntil, "count_until")
    
    # Wait for action server, send a goal, and register a callback for the response
    def send_goal(self, target_number, delay):
        # wait for the server. without a timeout, it will block indefinitely
        self.count_until_client_.wait_for_server()
        goal = CountUntil.Goal()
        
        # fill in goal fields. any filed omitted will be set to default values
        goal.target_number = target_number
        goal.delay = delay
        
        # send_goal_async() returns a Python Future object.
        # Register a callback for the goal’s response.
        # async allows method will return and we won’t block the execution. 
        # If we were to block the execution, we would also block the spin, 
        # and thus we would never get any response.
        self.count_until_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)

    # Get the goal response and if accepted, register a callback for the result
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(
                self.goal_result_callback)
        else:
            self.get_logger().info("Goal got rejected")

    # Get the goal result and print it
    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Result: " + str(result.reached_number))
    
    # Get the goal feedback and print it
    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_number
        self.get_logger().info("Got feedback: " + str(number))
        # if number >= 2:
        #    self.cancel_goal()

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(5, 0.5)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()