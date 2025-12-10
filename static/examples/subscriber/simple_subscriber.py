#!/usr/bin/env python3

"""
Basic ROS 2 subscriber example.

This node subscribes to a topic and receives messages.
It demonstrates the fundamental concept of subscribers in ROS 2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """A simple subscriber node that receives messages from a topic."""

    def __init__(self):
        """Initialize the subscriber node."""
        super().__init__('simple_subscriber')
        
        # Create a subscription that will receive String messages from the 'topic' topic
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)  # QoS profile depth
        
        # Prevent unused variable warning
        self.subscription  # type: ignore

    def listener_callback(self, msg):
        """Callback method that is called when a message is received."""
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Main function that initializes the node and starts spinning."""
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the subscriber node
    simple_subscriber = SimpleSubscriber()

    # Start spinning to execute callbacks
    rclpy.spin(simple_subscriber)

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    simple_subscriber.destroy_node()
    
    # Shutdown the rclpy library
    rclpy.shutdown()


if __name__ == '__main__':
    main()