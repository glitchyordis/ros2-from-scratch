#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# importing interface we need the 
# package name: example_interfaces
# folder for the topic messages: msg
# class for interface: Int64
from example_interfaces.msg import Int64

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.number_ = 2
        
        # publsih a topic named "number" of type Int64
        # with a queue size of 10
        self.number_publisher_ = \
            self.create_publisher(Int64, "number", 10)
            
        # publish every second
        self.timer_ = self.create_timer(1, self.publish_number)
        self.get_logger().info("Number publisher node started")
    
    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()