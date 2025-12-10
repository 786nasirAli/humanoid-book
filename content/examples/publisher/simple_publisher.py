#!/usr/bin/env python3

"""
Basic ROS 2 publisher example.

This node publishes a simple message to a topic at regular intervals.
It demonstrates the fundamental concept of publishers in ROS 2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """A simple publisher node that sends messages to a topic."""

    def __init__(self):
        """Initialize the publisher node."""
        super().__init__('simple_publisher')
        
        # Create a publisher that will send String messages to the 'topic' topic
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # Create a timer that calls the timer_callback method every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize a counter
        self.i = 0

    def timer_callback(self):
        """Callback method that publishes a message at regular intervals."""
        # Create a String message
        msg = String()
        msg.data = f'Hello World: {self.i}'
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        # Increment the counter
        self.i += 1


def main(args=None):
    """Main function that initializes the node and starts spinning."""
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the publisher node
    simple_publisher = SimplePublisher()

    # Start spinning to execute callbacks
    rclpy.spin(simple_publisher)

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    simple_publisher.destroy_node()
    
    # Shutdown the rclpy library
    rclpy.shutdown()


if __name__ == '__main__':
    main()