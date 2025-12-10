#!/usr/bin/env python3

"""
Simple test to verify publisher-subscriber communication.

This script does not run a full test but provides a simple way to verify
that publisher and subscriber nodes can communicate properly.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time


class PublisherSubscriberTest(Node):
    """Test node that acts as both publisher and subscriber."""
    
    def __init__(self):
        """Initialize the test node."""
        super().__init__('pubsub_test')
        
        # Create publisher and subscriber
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'test_topic',
            self.test_callback,
            10
        )
        
        self.received_messages = []
        self.test_completed = False
        self.message_count = 0
        self.max_messages = 5
        
        self.get_logger().info("Publisher-Subscriber test initialized")

    def test_callback(self, msg):
        """Handle received test messages."""
        self.received_messages.append(msg.data)
        self.get_logger().info(f"Received test message: {msg.data}")
        
        if len(self.received_messages) >= self.max_messages:
            self.test_completed = True

    def run_test(self):
        """Run the test by publishing messages and verifying reception."""
        # Publish test messages at 1Hz
        while not self.test_completed and rclpy.ok():
            msg = String()
            msg.data = f'Test message {self.message_count}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
            
            self.message_count += 1
            time.sleep(1)
            
            if self.message_count >= self.max_messages:
                break
        
        # Print test results
        self.get_logger().info(f"Test completed. Received {len(self.received_messages)} messages out of {self.max_messages} sent.")
        for i, msg in enumerate(self.received_messages):
            self.get_logger().info(f"  {i+1}: {msg}")
        
        if len(self.received_messages) == self.max_messages:
            self.get_logger().info("SUCCESS: All messages were received correctly!")
        else:
            self.get_logger().info("FAILURE: Some messages were not received.")


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    test_node = PublisherSubscriberTest()
    
    # Run test in a separate thread to allow ROS to process messages
    test_thread = threading.Thread(target=test_node.run_test)
    test_thread.start()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("Test interrupted by user")
    finally:
        test_thread.join(timeout=2)  # Wait for test to complete
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()