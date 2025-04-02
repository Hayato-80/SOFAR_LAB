#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from my_msgs.msg import MyInputMsg
from my_msgs.msg import MyOutputMsg

class MyNodePy(Node):

    def __init__(self):
        super().__init__('my_node_py')
        
        self.my_subscription = self.create_subscription(
            MyInputMsg,
            'my_input_topic',
            self.my_input_callback,
            10)
        self.my_subscription  # prevent unused variable warning
       
        self.my_publisher = self.create_publisher(MyOutputMsg, 'my_output_topic', 10)
        
    def my_input_callback(self, in_msg):
        
        out_msg = MyOutputMsg()
        self.my_publisher.publish(out_msg)
            
            
def main(args=None):
    rclpy.init(args=args)
    my_node_py = MyNodePy()
    rclpy.spin(my_node_py)
    simulation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
