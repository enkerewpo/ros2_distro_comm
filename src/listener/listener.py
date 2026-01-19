#!/usr/bin/env python3
"""
ROS2 Listener Node - Subscriber for testing cross-distribution communication.
Subscribes to 'chatter' topic and logs received messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os


class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        
        # Get ROS distribution name from environment
        self.ros_distro = os.environ.get('ROS_DISTRO', 'unknown')
        
        # Create subscription
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info(f'Listener node started on ROS2 {self.ros_distro}')
        self.get_logger().info('Waiting for messages on topic "chatter"...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received on {self.ros_distro}: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    listener = ListenerNode()
    
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
