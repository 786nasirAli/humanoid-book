#!/usr/bin/env python3

"""
Integration test for agent-controller communication in ROS 2.

This script tests the complete communication flow between agent and 
controller nodes for a humanoid robot, validating that commands are
sent, received, processed, and acknowledged properly.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from threading import Thread
import time
import math


class TestAgent(Node):
    """Test agent that sends commands and tracks responses."""

    def __init__(self):
        """Initialize the test agent node."""
        super().__init__('test_agent')
        
        # Publishers
        self.joint_cmd_publisher = self.create_publisher(JointState, '/test_joint_commands', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/test_cmd_vel', 10)
        
        # Subscription for responses
        self.response_subscription = self.create_subscription(
            String,
            '/test_response',
            self.response_callback,
            10
        )

        self.sent_commands = []
        self.received_responses = []
        self.command_counter = 0

    def send_command(self, cmd_type, value):
        """Send a test command."""
        if cmd_type == 'joint':
            # Create and send joint command
            joint_cmd = JointState()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.name = ['test_joint_1', 'test_joint_2']
            joint_cmd.position = [value, -value]
            
            self.joint_cmd_publisher.publish(joint_cmd)
            cmd_id = f'joint_cmd_{self.command_counter}'
            self.sent_commands.append(cmd_id)
            self.get_logger().info(f'Sent joint command: {cmd_id} with value {value}')
            
        elif cmd_type == 'base':
            # Create and send base command
            twist_cmd = Twist()
            twist_cmd.linear.x = value
            twist_cmd.angular.z = value * 0.1
            
            self.cmd_vel_publisher.publish(twist_cmd)
            cmd_id = f'base_cmd_{self.command_counter}'
            self.sent_commands.append(cmd_id)
            self.get_logger().info(f'Sent base command: {cmd_id} with value {value}')
        
        self.command_counter += 1
        return cmd_id

    def response_callback(self, msg):
        """Handle responses from the controller."""
        self.received_responses.append(msg.data)
        self.get_logger().info(f'Received response: {msg.data}')


class TestController(Node):
    """Test controller that receives commands and sends responses."""

    def __init__(self):
        """Initialize the test controller node."""
        super().__init__('test_controller')
        
        # Subscriptions
        self.joint_cmd_subscription = self.create_subscription(
            JointState,
            '/test_joint_commands',
            self.joint_cmd_callback,
            10
        )
        
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/test_cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for responses
        self.response_publisher = self.create_publisher(String, '/test_response', 10)

    def joint_cmd_callback(self, msg):
        """Handle received joint commands."""
        self.get_logger().info(f'Controller received joint command for {len(msg.name)} joints')
        
        # Validate the command
        success = len(msg.name) > 0 and len(msg.position) == len(msg.name)
        
        # Send response
        response = String()
        if success:
            response.data = f'Processed joint command for joints: {", ".join(msg.name)}'
        else:
            response.data = 'Error: Invalid joint command'
        
        self.response_publisher.publish(response)

    def cmd_vel_callback(self, msg):
        """Handle received base commands."""
        self.get_logger().info(f'Controller received base command: linear.x={msg.linear.x:.2f}')
        
        # Validate the command
        success = abs(msg.linear.x) <= 1.0  # Reasonable velocity limit
        
        # Send response
        response = String()
        if success:
            response.data = f'Processed base command: linear.x={msg.linear.x:.2f}'
        else:
            response.data = 'Error: Invalid base command'
        
        self.response_publisher.publish(response)


def run_integration_test():
    """Run the complete agent-controller integration test."""
    print("Starting Agent-Controller Integration Test...")
    print("=" * 50)
    
    # Initialize ROS 2
    rclpy.init()
    
    # Create nodes
    test_agent = TestAgent()
    test_controller = TestController()
    
    # Function to run the agent
    def run_agent():
        try:
            # Send a sequence of commands
            time.sleep(1)  # Wait for connections
            test_agent.send_command('joint', 1.0)
            time.sleep(1)
            test_agent.send_command('base', 0.5)
            time.sleep(1)
            test_agent.send_command('joint', 0.7)
            time.sleep(1)
            test_agent.send_command('base', -0.3)
            time.sleep(2)  # Wait for responses
        except KeyboardInterrupt:
            pass

    # Start agent in a separate thread
    agent_thread = Thread(target=run_agent)
    agent_thread.start()
    
    # Run the controller in the main thread
    try:
        rclpy.spin(test_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Wait for agent thread to finish
        agent_thread.join()
        
        # Print test results
        print("\n" + "=" * 50)
        print("INTEGRATION TEST RESULTS")
        print("=" * 50)
        print(f"Commands sent: {len(test_agent.sent_commands)}")
        print(f"Responses received: {len(test_agent.received_responses)}")
        print(f"Expected responses: {len(test_agent.sent_commands)}")
        
        success = len(test_agent.received_responses) == len(test_agent.sent_commands)
        
        print(f"\nCommands sent: {test_agent.sent_commands}")
        print(f"Responses received: {test_agent.received_responses}")
        
        if success:
            print("\n✅ Integration Test PASSED: All commands received responses!")
        else:
            print("\n❌ Integration Test FAILED: Some commands did not receive responses!")
        
        # Cleanup
        test_agent.destroy_node()
        test_controller.destroy_node()
        rclpy.shutdown()
        
        return success


def main(args=None):
    """Main function to run the integration test."""
    success = run_integration_test()
    return 0 if success else 1


if __name__ == '__main__':
    main()