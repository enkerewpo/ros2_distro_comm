#!/usr/bin/env python3
"""
ROS2 Listener Node - Subscriber for testing cross-distribution communication.
Subscribes to 'chatter' topic and logs received messages.
Performs node discovery checks before subscribing.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import sys
import time

# Add parent directory to path to import diagnostics
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from diagnostics import print_all_diagnostics


class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        
        # Get ROS distribution name from environment
        self.ros_distro = os.environ.get('ROS_DISTRO', 'unknown')
        
        self.get_logger().info(f'Listener node started on ROS2 {self.ros_distro}')
        
        # Perform node discovery check
        self.perform_node_discovery()
        
        # Create subscription
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info('Waiting for messages on topic "chatter"...')
    
    def perform_node_discovery(self):
        """Perform preliminary checks to discover other nodes."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Performing Node Discovery Checks')
        self.get_logger().info('=' * 60)
        
        max_retries = 10
        retry_delay = 1.0  # seconds
        
        for attempt in range(max_retries):
            # Get list of all nodes
            node_names = self.get_node_names()
            
            self.get_logger().info(f'Discovery attempt {attempt + 1}/{max_retries}')
            self.get_logger().info(f'Discovered nodes: {node_names}')
            
            # Check if talker node is present
            if '/talker' in node_names:
                self.get_logger().info('✓ Successfully discovered talker node!')
                
                # Get topics
                topic_names_and_types = self.get_topic_names_and_types()
                self.get_logger().info(f'Available topics: {[t[0] for t in topic_names_and_types]}')
                
                # Check if chatter topic exists
                chatter_exists = any(t[0] == '/chatter' for t in topic_names_and_types)
                if chatter_exists:
                    self.get_logger().info('✓ Successfully discovered /chatter topic!')
                else:
                    self.get_logger().warn('⚠ /chatter topic not yet available')
                
                self.get_logger().info('=' * 60)
                return
            else:
                self.get_logger().warn(f'⚠ Talker node not found yet, retrying in {retry_delay}s...')
                time.sleep(retry_delay)
        
        # If we get here, we didn't find the talker node
        self.get_logger().error('✗ Failed to discover talker node after multiple attempts')
        self.get_logger().error('  This may indicate a communication problem between containers')
        self.get_logger().info('=' * 60)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received on {self.ros_distro}: "{msg.data}"')


def main(args=None):
    # Print diagnostic information before starting
    print_all_diagnostics()
    
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
