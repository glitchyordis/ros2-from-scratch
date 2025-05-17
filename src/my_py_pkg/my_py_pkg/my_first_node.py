#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyCustomNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.counter_ = 0
        self.timer = self.create_timer(1, self.print_hello)
        
    def print_hello(self):
        self.get_logger().info(f'Hello {self.counter_}!')
        self.counter_ += 1
        
def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()