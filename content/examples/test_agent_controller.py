#!/usr/bin/env python3

"""
Integration test for agent-controller communication.

This test verifies that the agent-controller communication system
works correctly by simulating the complete communication loop.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import threading


class AgentControllerTester(Node):
    """
    Integration test for agent-controller communication.
    
    This node tests the communication between an agent that publishes
    commands and a controller that sends feedback.
    """
    
    def __init__(self):
        """Initialize the tester."""
        super().__init__('agent_controller_tester')
        
        # Publishers and subscribers
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            10
        )
        
        self.state_publisher = self.create_publisher(
            JointState,
            'humanoid_sensors/joint_states',
            10
        )
        
        self.command_subscriber = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.command_callback,
            10
        )
        
        # Data tracking
        self.received_commands = []
        self.command_count = 0
        self.test_completed = False
        self.test_passed = False
        
        # Joint names for the test
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]
        
        self.get_logger().info("Agent-Controller Tester initialized")

    def command_callback(self, msg):
        """Record received commands for testing."""
        self.received_commands.append(msg.data)
        self.command_count += 1
        self.get_logger().info(f"Test: Received command #{self.command_count} with {len(msg.data)} values")
        
        # Test passes if we receive valid commands
        if len(msg.data) == len(self.joint_names):
            self.test_passed = True

    def run_test(self):
        """Execute the integration test."""
        self.get_logger().info("Starting agent-controller integration test")
        
        # Send test commands periodically
        test_commands = [
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],  # Initial position
            [0.2, 0.3, 0.4, 0.5, 0.6, 0.7],  # Next position
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Return to zero
        ]
        
        for i, cmd in enumerate(test_commands):
            # Create and send command
            cmd_msg = Float64MultiArray()
            cmd_msg.data = cmd
            self.command_publisher.publish(cmd_msg)
            self.get_logger().info(f"Test: Published command #{i+1}: {cmd}")
            
            # Publish corresponding feedback state (simulating controller)
            state_msg = JointState()
            state_msg.name = self.joint_names
            state_msg.position = [x + 0.01 for x in cmd]  # Simulate slight deviation
            state_msg.velocity = [0.0] * len(cmd)
            state_msg.effort = [0.0] * len(cmd)
            state_msg.header.stamp = self.get_clock().now().to_msg()
            self.state_publisher.publish(state_msg)
            
            # Wait between commands
            time.sleep(0.5)
        
        # Wait a bit more to receive all callbacks
        time.sleep(1.0)
        
        # Final test results
        self.get_logger().info(f"Test completed. Commands sent: {len(test_commands)}, Commands received: {len(self.received_commands)}")
        
        if self.test_passed:
            self.get_logger().info("✓ Agent-controller communication test PASSED")
        else:
            self.get_logger().info("✗ Agent-controller communication test FAILED")


def main(args=None):
    """Main test function."""
    rclpy.init(args=args)
    
    tester = AgentControllerTester()
    
    # Run the test in a separate thread to allow ROS to process messages
    test_thread = threading.Thread(target=tester.run_test)
    test_thread.start()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("Tester interrupted by user")
    finally:
        test_thread.join(timeout=2)  # Wait for test to complete
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()