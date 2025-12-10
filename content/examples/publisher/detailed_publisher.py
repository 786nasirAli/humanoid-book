#!/usr/bin/env python3

"""
Detailed ROS 2 publisher example for humanoid robot sensors.

This node simulates publishing sensor data from a humanoid robot,
demonstrating more advanced publisher concepts like different
message types, Quality of Service settings, and proper node structure.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Header
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
import math
import random


class DetailedPublisher(Node):
    """
    A detailed publisher node simulating various humanoid robot sensors.
    
    This publisher demonstrates:
    - Different message types (JointState, Float64MultiArray)
    - Quality of Service settings
    - Simulated sensor data generation
    - Proper message structure following ROS 2 standards
    """
    
    def __init__(self):
        """Initialize the detailed publisher node."""
        super().__init__('detailed_publisher')
        
        # Create publishers for different types of data
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            'humanoid_joint_states', 
            10  # QoS history depth
        )
        
        self.imu_data_publisher = self.create_publisher(
            Float64MultiArray, 
            'imu_data', 
            10
        )
        
        # Create a timer that triggers our callbacks
        timer_period = 0.1  # seconds (10Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize sensor data
        self.joint_names = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle',
            'left_shoulder', 'left_elbow',
            'right_shoulder', 'right_elbow'
        ]
        self.position_data = [0.0] * len(self.joint_names)
        self.velocity_data = [0.0] * len(self.joint_names)
        self.effort_data = [0.0] * len(self.joint_names)
        
        self.get_logger().info("Detailed Publisher initialized")

    def timer_callback(self):
        """Callback method that publishes various sensor messages at regular intervals."""
        # Generate simulated joint state data
        self.update_joint_positions()
        
        # Publish joint states
        joint_msg = self.create_joint_state_message()
        self.joint_state_publisher.publish(joint_msg)
        
        # Publish IMU data
        imu_msg = self.create_imu_message()
        self.imu_data_publisher.publish(imu_msg)
        
        self.get_logger().info(f"Published joint states for {len(joint_msg.name)} joints")

    def update_joint_positions(self):
        """Simulate changing joint positions over time."""
        time_sec = self.get_clock().now().nanoseconds / 1e9
        
        for i in range(len(self.joint_names)):
            # Create a slightly different oscillation for each joint
            amplitude = 0.5
            frequency = 0.5 + (i * 0.1)
            phase = i * 0.5
            
            self.position_data[i] = amplitude * math.sin(frequency * time_sec + phase)
            # Simple velocity approximation (derivative of position)
            self.velocity_data[i] = amplitude * frequency * math.cos(frequency * time_sec + phase)
            # Simple effort approximation (proportional to position)
            self.effort_data[i] = -1.0 * self.position_data[i]

    def create_joint_state_message(self):
        """Create and populate a JointState message."""
        msg = JointState()
        
        # Set header with timestamp
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'humanoid_base'
        
        # Set joint names and data
        msg.name = self.joint_names
        msg.position = self.position_data
        msg.velocity = self.velocity_data
        msg.effort = self.effort_data
        
        return msg

    def create_imu_message(self):
        """Create and populate an IMU data message."""
        msg = Float64MultiArray()
        
        # Simulate IMU data: [orientation_x, orientation_y, orientation_z, orientation_w,
        #                     angular_velocity_x, angular_velocity_y, angular_velocity_z,
        #                     linear_acceleration_x, linear_acceleration_y, linear_acceleration_z]
        time_sec = self.get_clock().now().nanoseconds / 1e9
        
        # Simulated orientation (quaternion components)
        orientation = [
            0.0,  # x
            0.1 * math.sin(0.5 * time_sec),  # y
            0.0,  # z
            math.cos(0.1 * time_sec)  # w
        ]
        
        # Simulated angular velocity
        angular_velocity = [
            0.05 * math.cos(0.5 * time_sec),  # x
            0.0,  # y
            0.0   # z
        ]
        
        # Simulated linear acceleration
        linear_acceleration = [
            0.2 * math.sin(0.3 * time_sec),  # x
            9.81,  # y (gravity)
            0.1 * math.cos(0.4 * time_sec)   # z
        ]
        
        # Combine all values into a single array
        msg.data = orientation + angular_velocity + linear_acceleration
        
        return msg


def main(args=None):
    """Main function that initializes the node and starts spinning."""
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the detailed publisher node
    detailed_publisher = DetailedPublisher()

    try:
        # Start spinning to execute callbacks
        rclpy.spin(detailed_publisher)
    except KeyboardInterrupt:
        detailed_publisher.get_logger().info("Interrupted by user")
    finally:
        # Destroy the node explicitly
        detailed_publisher.destroy_node()
        
        # Shutdown the rclpy library
        rclpy.shutdown()


if __name__ == '__main__':
    main()