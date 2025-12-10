#!/usr/bin/env python3

"""
Python agent example that publishes commands to humanoid robot controllers.

This node demonstrates how a Python agent can send commands to ROS 2 controllers
for a humanoid robot, using appropriate message types and QoS settings.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
import math
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


class AgentPublisher(Node):
    """A Python agent that publishes commands to humanoid robot controllers."""

    def __init__(self):
        """Initialize the agent publisher node."""
        super().__init__('agent_publisher')
        
        # Create QoS profile for real-time control
        qos_profile = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(JointState, '/joint_commands', qos_profile)
        
        # Publisher for base movement commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        
        # Publisher for status updates
        self.status_publisher = self.create_publisher(String, '/agent_status', 10)

        # Create a timer that calls the timer_callback method at 20Hz
        timer_period = 0.05  # seconds (20Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter
        self.i = 0
        
        # Define joint names for a simple humanoid
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        self.get_logger().info('Agent Publisher node initialized')

    def timer_callback(self):
        """Callback method that publishes commands at regular intervals."""
        # Publish joint commands
        joint_cmd_msg = JointState()
        joint_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        joint_cmd_msg.header.frame_id = 'base_link'
        joint_cmd_msg.name = self.joint_names
        
        # Generate joint command positions using oscillating patterns
        positions = []
        velocities = []
        efforts = []
        
        for idx, name in enumerate(self.joint_names):
            # Different motion patterns for different joint types
            if 'hip' in name:
                pos = math.sin(self.i / 10.0 + idx) * 0.3
                vel = 0.0
                eff = 0.0
            elif 'knee' in name:
                pos = math.sin(self.i / 8.0 + idx) * 0.2
                vel = 0.0
                eff = 0.0
            elif 'ankle' in name:
                pos = math.sin(self.i / 12.0 + idx) * 0.1
                vel = 0.0
                eff = 0.0
            elif 'shoulder' in name:
                pos = math.sin(self.i / 15.0 + idx) * 0.4
                vel = 0.0
                eff = 0.0
            else:  # elbow
                pos = math.sin(self.i / 7.0 + idx) * 0.35
                vel = 0.0
                eff = 0.0

            positions.append(pos)
            velocities.append(vel)
            efforts.append(eff)

        joint_cmd_msg.position = positions
        joint_cmd_msg.velocity = velocities
        joint_cmd_msg.effort = efforts

        self.joint_cmd_publisher.publish(joint_cmd_msg)
        self.get_logger().info(f'Published joint commands for {len(self.joint_names)} joints')

        # Publish base movement command (e.g., for a mobile base)
        twist_msg = Twist()
        # Simple movement pattern: move forward and rotate
        twist_msg.linear.x = 0.2  # Move forward at 0.2 m/s
        twist_msg.angular.z = math.sin(self.i / 20.0) * 0.3  # Oscillating rotation
        
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info(f'Published base command: linear.x={twist_msg.linear.x:.2f}, angular.z={twist_msg.angular.z:.2f}')

        # Publish status message
        status_msg = String()
        status_msg.data = f'Agent status: {self.i}, publishing commands'
        self.status_publisher.publish(status_msg)

        # Increment the counter
        self.i += 1


def main(args=None):
    """Main function that initializes the node and starts spinning."""
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the agent publisher node
    agent_publisher = AgentPublisher()

    # Start spinning to execute callbacks
    try:
        rclpy.spin(agent_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        agent_publisher.destroy_node()

        # Shutdown the rclpy library
        rclpy.shutdown()


if __name__ == '__main__':
    main()