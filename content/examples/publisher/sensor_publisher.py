#!/usr/bin/env python3

"""
Humanoid robot sensor publisher example.

This node simulates publishing sensor data from a humanoid robot,
specifically focusing on joint state sensors which are critical
for humanoid robot control and awareness.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import random


class SensorPublisher(Node):
    """A publisher that simulates humanoid robot sensor data."""
    
    def __init__(self):
        """Initialize the sensor publisher."""
        super().__init__('humanoid_sensor_publisher')
        
        # Create publisher for joint states
        self.publisher_ = self.create_publisher(JointState, 'humanoid_sensors/joint_states', 10)
        
        # Timer to publish data at 50Hz (every 20ms)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize joint names for a simple humanoid
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'torso_joint', 'neck_joint'
        ]
        
        # Initialize positions, velocities, and efforts
        self.positions = [0.0] * len(self.joint_names)
        self.velocities = [0.0] * len(self.joint_names)
        self.efforts = [0.0] * len(self.joint_names)
        
        self.get_logger().info(f"Sensor Publisher initialized with {len(self.joint_names)} joints")

    def timer_callback(self):
        """Publish joint state data."""
        # Create and populate the JointState message
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.name = self.joint_names
        msg.position = self.positions.copy()
        msg.velocity = self.velocities.copy()
        msg.effort = self.efforts.copy()
        
        # Simulate changing joint positions (simple oscillation for demonstration)
        current_time = self.get_clock().now().nanoseconds / 1e9
        for i in range(len(self.joint_names)):
            # Different oscillation patterns for different joint types
            if 'hip' in self.joint_names[i]:
                self.positions[i] = 0.2 * math.sin(0.5 * current_time + i)
            elif 'knee' in self.joint_names[i]:
                self.positions[i] = 0.15 * math.sin(0.6 * current_time + i)
            elif 'ankle' in self.joint_names[i]:
                self.positions[i] = 0.1 * math.sin(0.7 * current_time + i)
            elif 'shoulder' in self.joint_names[i]:
                self.positions[i] = 0.25 * math.sin(0.4 * current_time + i)
            elif 'elbow' in self.joint_names[i]:
                self.positions[i] = 0.3 * math.sin(0.8 * current_time + i)
            else:  # torso, neck
                self.positions[i] = 0.1 * math.sin(0.3 * current_time + i)
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published joint states for {len(msg.name)} joints")


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    sensor_publisher = SensorPublisher()
    
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        sensor_publisher.get_logger().info("Sensor publisher stopped by user")
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()