#!/usr/bin/env python3
"""
ROS2 Talker Node - Publisher for testing cross-distribution communication.
Publishes string messages to 'chatter' topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import sys

# Add parent directory to path to import diagnostics
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from diagnostics import print_all_diagnostics


class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        
        # Get ROS distribution name from environment
        self.ros_distro = os.environ.get('ROS_DISTRO', 'unknown')
        
        # Create publisher
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # Create timer to publish messages every second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info(f'Talker node started on ROS2 {self.ros_distro}')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from {self.ros_distro}! Message #{self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1


def main(args=None):
    # Print diagnostic information before starting
    print_all_diagnostics()
    
    rclpy.init(args=args)
    talker = TalkerNode()
    
    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
