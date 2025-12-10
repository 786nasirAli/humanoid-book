#!/usr/bin/env python3

"""
Python agent publisher example.

This node simulates a high-level AI agent that publishes commands
to control a humanoid robot. This demonstrates the bridge between
high-level decision making (typically done in Python) and the ROS 2
control system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import time
import math


class AgentPublisher(Node):
    """
    A Python agent that publishes commands to control a humanoid robot.
    
    This example simulates an agent that sends joint position commands
    to make the robot perform a simple movement pattern.
    """
    
    def __init__(self):
        """Initialize the agent publisher."""
        super().__init__('agent_publisher')
        
        # Publisher for joint commands
        self.command_publisher = self.create_publisher(
            Float64MultiArray, 
            'joint_commands', 
            10
        )
        
        # Subscriber for current joint states (for feedback/monitoring)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'humanoid_sensors/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Timer for publishing commands at regular intervals
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize command data
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]
        
        self.current_positions = [0.0] * len(self.joint_names)
        self.time_offset = 0.0
        
        self.get_logger().info(f"Agent Publisher initialized for {len(self.joint_names)} joints")

    def joint_state_callback(self, msg):
        """Receive and store current joint states."""
        # Update current positions for joints we're controlling
        for i, name in enumerate(self.joint_names):
            try:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
            except ValueError:
                # Joint not found in message
                pass

    def timer_callback(self):
        """Publish joint commands based on a pattern."""
        # Update time offset for pattern generation
        self.time_offset += 0.1
        
        # Generate command values based on a pattern
        command_msg = Float64MultiArray()
        
        # Create oscillating patterns for different joint groups
        commands = []
        for i, joint_name in enumerate(self.joint_names):
            if 'hip' in joint_name:
                # Hip joints move in opposition for walking simulation
                if 'left' in joint_name:
                    cmd = 0.3 * math.sin(self.time_offset)
                else:  # right
                    cmd = 0.3 * math.sin(self.time_offset + math.pi)
            elif 'knee' in joint_name:
                # Knee joints follow hip pattern with phase shift
                if 'left' in joint_name:
                    cmd = 0.25 * math.sin(self.time_offset + math.pi/4)
                else:  # right
                    cmd = 0.25 * math.sin(self.time_offset + math.pi + math.pi/4)
            elif 'shoulder' in joint_name:
                # Shoulder joints move together
                cmd = 0.2 * math.sin(self.time_offset * 0.7)
            elif 'elbow' in joint_name:
                # Elbow joints move in opposition to shoulders
                cmd = 0.15 * math.sin(self.time_offset * 0.7 + math.pi)
            else:
                # Default: small oscillation
                cmd = 0.1 * math.sin(self.time_offset * 0.5)
            
            commands.append(cmd)
        
        command_msg.data = commands
        
        # Publish the command
        self.command_publisher.publish(command_msg)
        
        # Log the first few commands
        self.get_logger().info(
            f"Published commands: "
            f"Left hip: {commands[0]:.3f}, "
            f"Right hip: {commands[3]:.3f}, "
            f"Left shoulder: {commands[6]:.3f}"
        )

    def get_current_positions(self):
        """Get the currently known joint positions."""
        return self.current_positions.copy()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    agent_publisher = AgentPublisher()
    
    try:
        rclpy.spin(agent_publisher)
    except KeyboardInterrupt:
        agent_publisher.get_logger().info("Agent publisher stopped by user")
    finally:
        agent_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()