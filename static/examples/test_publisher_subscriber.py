#!/usr/bin/env python3

"""
Test script for publisher-subscriber communication in ROS 2.

This script tests the communication between publisher and subscriber nodes,
validating that messages are transmitted correctly between ROS 2 nodes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread
import time


class TestPublisher(Node):
    """A test publisher to validate communication."""

    def __init__(self):
        """Initialize the test publisher node."""
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        self.i = 0

    def publish_message(self):
        """Publish a single message."""
        msg = String()
        msg.data = f'Test message {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1


class TestSubscriber(Node):
    """A test subscriber to validate communication."""

    def __init__(self):
        """Initialize the test subscriber node."""
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            String,
            'test_topic',
            self.listener_callback,
            10)
        self.received_messages = []
        self.expected_messages = []

    def listener_callback(self, msg):
        """Process received messages."""
        self.get_logger().info(f'Received: "{msg.data}"')
        self.received_messages.append(msg.data)


def run_test():
    """Run the publisher-subscriber communication test."""
    print("Starting publisher-subscriber communication test...")
    
    # Initialize ROS 2
    rclpy.init()
    
    # Create publisher and subscriber nodes
    publisher_node = TestPublisher()
    subscriber_node = TestSubscriber()
    
    # Expected messages to be sent
    subscriber_node.expected_messages = [f'Test message {i}' for i in range(5)]
    
    # Function to run the publisher
    def run_publisher():
        for i in range(5):
            publisher_node.publish_message()
            time.sleep(0.5)  # Wait between messages
    
    # Start publisher in a separate thread
    publisher_thread = Thread(target=run_publisher)
    publisher_thread.start()
    
    # Spin the subscriber to process messages
    try:
        while len(subscriber_node.received_messages) < 5:
            rclpy.spin_once(subscriber_node, timeout_sec=1.0)
            if not publisher_thread.is_alive() and len(subscriber_node.received_messages) < 5:
                break
    except KeyboardInterrupt:
        pass
    
    # Wait for publisher thread to finish
    publisher_thread.join()
    
    # Print results
    print(f"\nExpected {len(subscriber_node.expected_messages)} messages")
    print(f"Received {len(subscriber_node.received_messages)} messages")
    print(f"Expected: {subscriber_node.expected_messages}")
    print(f"Received: {subscriber_node.received_messages}")
    
    # Validate results
    success = len(subscriber_node.received_messages) == len(subscriber_node.expected_messages)
    
    if success:
        for expected, received in zip(subscriber_node.expected_messages, subscriber_node.received_messages):
            if expected != received:
                success = False
                break
    
    if success:
        print("\n✅ Test PASSED: All messages were received correctly!")
    else:
        print("\n❌ Test FAILED: Some messages were not received correctly!")
    
    # Cleanup
    publisher_node.destroy_node()
    subscriber_node.destroy_node()
    rclpy.shutdown()
    
    return success


def main(args=None):
    """Main function to run the test."""
    success = run_test()
    if success:
        return 0
    else:
        return 1


if __name__ == '__main__':
    main()