#!/usr/bin/env python3

"""
Detailed ROS 2 publisher example for humanoid robot.

This node publishes detailed messages to topics at regular intervals.
It demonstrates advanced publisher concepts in ROS 2 including custom
message types and QoS settings optimized for humanoid robots.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import math


class DetailedPublisher(Node):
    """A detailed publisher node demonstrating advanced ROS 2 concepts."""

    def __init__(self):
        """Initialize the detailed publisher node."""
        super().__init__('detailed_publisher')

        # Create a publisher with custom QoS settings for real-time joint control
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', qos_profile)
        
        # Create a string publisher for status messages
        self.status_publisher = self.create_publisher(String, 'status', 10)

        # Create a timer that calls the timer_callback method every 0.1 seconds (10Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter
        self.i = 0

        # Initialize joint names for a simple humanoid
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        self.get_logger().info('Detailed Publisher node initialized')

    def timer_callback(self):
        """Callback method that publishes detailed messages at regular intervals."""
        
        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'
        joint_msg.name = self.joint_names
        
        # Generate simulated joint positions using oscillating patterns
        positions = []
        velocities = []
        efforts = []
        
        for idx, name in enumerate(self.joint_names):
            # Different motion patterns for different joint types
            if 'hip' in name:
                pos = math.sin(self.i / 10.0 + idx) * 0.5
                vel = math.cos(self.i / 10.0 + idx) * 0.5 / 10.0
            elif 'knee' in name:
                pos = math.sin(self.i / 8.0 + idx) * 0.3
                vel = math.cos(self.i / 8.0 + idx) * 0.3 / 8.0
            elif 'ankle' in name:
                pos = math.sin(self.i / 12.0 + idx) * 0.2
                vel = math.cos(self.i / 12.0 + idx) * 0.2 / 12.0
            elif 'shoulder' in name:
                pos = math.sin(self.i / 15.0 + idx) * 0.4
                vel = math.cos(self.i / 15.0 + idx) * 0.4 / 15.0
            else:  # elbow
                pos = math.sin(self.i / 7.0 + idx) * 0.35
                vel = math.cos(self.i / 7.0 + idx) * 0.35 / 7.0

            positions.append(pos)
            velocities.append(vel)
            efforts.append(0.0)  # For now, no actual effort

        joint_msg.position = positions
        joint_msg.velocity = velocities
        joint_msg.effort = efforts

        self.joint_publisher.publish(joint_msg)
        self.get_logger().info(f'Published joint states for {len(self.joint_names)} joints')

        # Publish status message
        status_msg = String()
        status_msg.data = f'Detailed publisher status: {self.i}'
        self.status_publisher.publish(status_msg)
        self.get_logger().info(f'Published status: "{status_msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """Main function that initializes the node and starts spinning."""
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the detailed publisher node
    detailed_publisher = DetailedPublisher()

    # Start spinning to execute callbacks
    try:
        rclpy.spin(detailed_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
        detailed_publisher.destroy_node()

        # Shutdown the rclpy library
        rclpy.shutdown()


if __name__ == '__main__':
    main()