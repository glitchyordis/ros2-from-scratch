#!/usr/bin/env python3
import rclpy

# importing interface we need the 
# package name: example_interfaces
# folder for the topic messages: msg
# class for interface: Int64
from example_interfaces.msg import Int64
from rclpy.node import Node
from rclpy.parameter import Parameter


class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        
        # parameters require a name and data type (depends on default val you provide)
        # String: self.declare_parameter("device_name", "/dev/ttyUSB0")
        # Int array: self.declare_parameter("numbers", [4, 5, 6])
        # declare param before getting it else you'll get arameterNotDeclaredException)
        self.declare_parameter("number", 2) # this means you must use int, not float else get InvalidParameterTypeException
        self.declare_parameter("publish_period", 1.0)        
        
        self.number_ = self.get_parameter("number").value
        self.timer_period_ = self.get_parameter("publish_period").value
        
        self.add_post_set_parameters_callback(self.parameters_callback)
        
        # publsih a topic named "number" of type Int64
        # with a queue size of 10
        self.number_publisher_ = \
            self.create_publisher(Int64, "number", 10)
            
        # publish every second
        self.number_timer_ = self.create_timer(self.timer_period_, self.publish_number)
        self.get_logger().info("Number publisher node started")
    
    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        
    def parameters_callback(self, params: list[Parameter]):
        #  For each parameter, you can access its name, value, and type.
        for param in params:
            if param.name == "number":
                self.number_ = param.value

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()