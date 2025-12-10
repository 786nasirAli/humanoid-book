#!/usr/bin/env python3

"""
Complete agent-controller communication example.

This example demonstrates a complete communication loop between
a Python AI agent and robot controllers in a humanoid system.
The agent makes high-level decisions based on sensor input and
sends commands to controllers which execute them on the robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import time
import math


class AgentControllerDemo(Node):
    """
    Complete demo of agent-controller communication for humanoid robot.
    
    This node combines both agent and controller functionality to demonstrate
    the complete loop: sensor data → agent decision → control command → 
    robot simulation → new sensor data.
    """
    
    def __init__(self):
        """Initialize the complete demo."""
        super().__init__('agent_controller_demo')
        
        # Publishers
        self.command_publisher = self.create_publisher(
            Float64MultiArray, 
            'joint_commands', 
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'humanoid_sensors/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Data storage
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]
        
        self.current_positions = [0.0] * len(self.joint_names)
        self.current_velocities = [0.0] * len(self.joint_names)
        self.current_efforts = [0.0] * len(self.joint_names)
        
        # Timing
        self.time_offset = 0.0
        
        # Control timer (50 Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # State tracking
        self.demo_state = "balance"  # balance, walk, gesture
        self.state_timer = 0.0
        
        self.get_logger().info("Agent-Controller Demo initialized")

    def joint_state_callback(self, msg):
        """Receive and store joint state."""
        for i, name in enumerate(self.joint_names):
            try:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    self.current_velocities[i] = msg.velocity[idx]
                if idx < len(msg.effort):
                    self.current_efforts[i] = msg.effort[idx]
            except ValueError:
                # Joint not found in message
                pass

    def control_loop(self):
        """Main control loop processing sensor data and generating commands."""
        # Update timing
        self.time_offset += 0.02  # Timer period
        self.state_timer += 0.02
        
        # Switch states based on time
        if self.state_timer > 10.0:  # Switch every 10 seconds
            if self.demo_state == "balance":
                self.demo_state = "walk"
            elif self.demo_state == "walk":
                self.demo_state = "gesture"
            else:
                self.demo_state = "balance"
            self.state_timer = 0.0
            self.get_logger().info(f"Switching to state: {self.demo_state}")
        
        # Process based on current state
        if self.demo_state == "balance":
            commands = self.balance_control()
        elif self.demo_state == "walk":
            commands = self.walk_pattern()
        else:  # gesture
            commands = self.gesture_pattern()
        
        # Publish joint commands
        command_msg = Float64MultiArray()
        command_msg.data = commands
        self.command_publisher.publish(command_msg)
        
        # Publish velocity command (for base movement)
        cmd_vel_msg = Twist()
        if self.demo_state == "walk":
            cmd_vel_msg.linear.x = 0.5  # Move forward slowly
        else:
            cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
        # Log current state periodically
        if int(self.time_offset * 10) % 5 == 0:  # Every 0.5 seconds
            self.get_logger().info(
                f"{self.demo_state} state - "
                f"LHip: {commands[0]:.3f}, "
                f"RHip: {commands[3]:.3f}, "
                f"LS: {commands[6]:.3f}, "
                f"RS: {commands[8]:.3f}"
            )

    def balance_control(self):
        """Generate balance control commands."""
        # Simple balance control: keep joints near center position
        commands = []
        for i, joint_name in enumerate(self.joint_names):
            # Base position (0 for most joints)
            base_pos = 0.0
            
            # Add small oscillations for lifelike behavior
            if 'hip' in joint_name:
                base_pos = 0.1 * math.sin(self.time_offset * 0.5)
            elif 'shoulder' in joint_name:
                base_pos = 0.05 * math.sin(self.time_offset * 0.7)
            
            commands.append(base_pos)
        
        return commands

    def walk_pattern(self):
        """Generate walking pattern commands."""
        # Generate a simple walking pattern
        commands = []
        for i, joint_name in enumerate(self.joint_names):
            if 'hip' in joint_name:
                # Hip joints move in opposition for walking
                if 'left' in joint_name:
                    cmd = 0.3 * math.sin(self.time_offset)
                else:  # right
                    cmd = 0.3 * math.sin(self.time_offset + math.pi)
            elif 'knee' in joint_name:
                # Knee joints follow hip with phase offset
                if 'left' in joint_name:
                    cmd = 0.25 * math.sin(self.time_offset + math.pi/3)
                else:  # right
                    cmd = 0.25 * math.sin(self.time_offset + math.pi + math.pi/3)
            elif 'shoulder' in joint_name:
                # Arms swing opposite to legs
                if 'left' in joint_name:
                    cmd = 0.2 * math.sin(self.time_offset + math.pi)
                else:  # right
                    cmd = 0.2 * math.sin(self.time_offset)
            elif 'elbow' in joint_name:
                # Elbows bend during walking
                cmd = 0.3 + 0.1 * math.sin(self.time_offset * 1.5)
            else:
                # Other joints stay near zero
                cmd = 0.0
            
            commands.append(cmd)
        
        return commands

    def gesture_pattern(self):
        """Generate gesture pattern commands."""
        # Generate a waving gesture pattern
        commands = []
        for i, joint_name in enumerate(self.joint_names):
            if 'shoulder' in joint_name:
                # Shoulder moves for waving
                if 'left' in joint_name:
                    # Left shoulder raises and waves
                    cmd = 0.8 + 0.3 * math.sin(self.time_offset * 3)
                else:  # right shoulder
                    # Right shoulder stays stable for support
                    cmd = 0.1
            elif 'elbow' in joint_name:
                if 'left' in joint_name:
                    # Left elbow bends for waving
                    cmd = 1.0 + 0.5 * math.sin(self.time_offset * 4)
                else:  # right elbow
                    cmd = 0.5  # keep right arm slightly bent
            elif 'hip' in joint_name:
                # Keep hips stable during gesture
                cmd = 0.05 * math.sin(self.time_offset * 0.3)
            else:
                # Other joints stay near zero
                cmd = 0.0
            
            commands.append(cmd)
        
        return commands


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    demo = AgentControllerDemo()
    
    try:
        rclpy.spin(demo)
    except KeyboardInterrupt:
        demo.get_logger().info("Agent-Controller demo stopped by user")
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()