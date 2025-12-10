#!/usr/bin/env python3

"""
Complete ROS 2 nervous system example for a humanoid robot.

This example demonstrates how all the components work together to form
a complete "nervous system" for a humanoid robot, integrating sensor
processing, decision making, and actuator control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist, Vector3
from builtin_interfaces.msg import Time

import math
import time
import threading


class HumanoidNervousSystem(Node):
    """
    Complete ROS 2 nervous system for a humanoid robot.
    
    This node demonstrates the integration of perception, decision-making,
    and action in a humanoid robot system.
    """
    
    def __init__(self):
        super().__init__('humanoid_nervous_system')
        
        # QoS profiles for different data types
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers for commands and feedback
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray, 'joint_commands', command_qos)
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 'cmd_vel', command_qos)
        
        self.status_publisher = self.create_publisher(
            String, 'robot_status', 10)
        
        # Subscribers for sensor data
        self.joint_state_subscriber = self.create_subscription(
            JointState, 'humanoid_sensors/joint_states', 
            self.joint_state_callback, sensor_qos)
        
        self.imu_subscriber = self.create_subscription(
            Imu, 'humanoid_sensors/imu', 
            self.imu_callback, sensor_qos)
        
        # Internal state
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'neck_joint'
        ]
        
        self.current_joint_positions = {name: 0.0 for name in self.joint_names}
        self.imu_data = {'orientation': Vector3(), 'angular_velocity': Vector3(), 'linear_acceleration': Vector3()}
        self.robot_state = 'idle'  # idle, walking, balancing, gesturing
        self.balance_error = 0.0
        
        # Control timer (100 Hz for real-time response)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        # State management timer (1 Hz for state changes)
        self.state_timer = self.create_timer(1.0, self.state_management)
        
        self.get_logger().info("Humanoid Nervous System initialized")

    def joint_state_callback(self, msg):
        """Process joint state messages (perception layer)."""
        for i, name in enumerate(msg.name):
            if name in self.current_joint_positions:
                if i < len(msg.position):
                    self.current_joint_positions[name] = msg.position[i]
        
        self.get_logger().debug(f"Updated {len(msg.name)} joints")

    def imu_callback(self, msg):
        """Process IMU data for balance control (perception layer)."""
        # Extract orientation from quaternion (simplified)
        # In a real system, you'd properly convert quaternion to euler angles
        self.imu_data['orientation'].x = msg.orientation.x
        self.imu_data['orientation'].y = msg.orientation.y
        self.imu_data['orientation'].z = msg.orientation.z
        
        self.imu_data['angular_velocity'] = msg.angular_velocity
        self.imu_data['linear_acceleration'] = msg.linear_acceleration
        
        # Calculate balance error from pitch (simplified)
        self.balance_error = abs(msg.orientation.y)  # pitch error
        
        self.get_logger().debug(f"IMU: pitch={msg.orientation.y:.3f}, balance_error={self.balance_error:.3f}")

    def state_management(self):
        """Manage high-level robot state (decision layer)."""
        # Simple state machine based on balance error
        if self.balance_error > 0.3:
            new_state = 'balancing'
        elif self.balance_error > 0.1:
            new_state = 'cautious'
        else:
            # Alternate between walking and gesturing
            if self.robot_state in ['idle', 'cautious', 'balancing']:
                new_state = 'walking'
            elif self.robot_state == 'walking':
                new_state = 'gesturing'
            else:
                new_state = 'idle'
        
        if new_state != self.robot_state:
            self.get_logger().info(f"State change: {self.robot_state} → {new_state}")
            self.robot_state = new_state
            
            # Publish status update
            status_msg = String()
            status_msg.data = f"State: {self.robot_state}, Balance Error: {self.balance_error:.3f}"
            self.status_publisher.publish(status_msg)

    def control_loop(self):
        """Main control loop integrating perception, decision, and action."""
        # Based on current state, generate appropriate commands
        if self.robot_state == 'balancing':
            commands = self.balance_control()
        elif self.robot_state == 'walking':
            commands = self.walk_pattern()
        elif self.robot_state == 'gesturing':
            commands = self.gesture_pattern()
        elif self.robot_state == 'cautious':
            commands = self.cautious_pattern()
        else:  # idle
            commands = self.idle_position()
        
        # Publish joint commands (action layer)
        cmd_msg = Float64MultiArray()
        cmd_msg.data = commands
        self.joint_command_publisher.publish(cmd_msg)
        
        # Publish velocity command if walking
        if self.robot_state == 'walking':
            vel_msg = Twist()
            vel_msg.linear.x = 0.3  # Move forward slowly
            vel_msg.angular.z = 0.1 * math.sin(self.get_clock().now().nanoseconds / 1e9)  # Gentle turning
            self.cmd_vel_publisher.publish(vel_msg)

    def balance_control(self):
        """Generate balance control commands."""
        commands = []
        for name in self.joint_names:
            if 'hip' in name:
                # Adjust hip position based on balance error
                correction = -2.0 * self.imu_data['orientation'].y
                commands.append(correction)
            elif 'knee' in name:
                # Knee follows hip with some offset
                correction = -1.5 * self.imu_data['orientation'].y
                commands.append(correction)
            elif 'ankle' in name:
                # Ankle for fine balance adjustment
                correction = -1.0 * self.imu_data['orientation'].y
                commands.append(correction)
            elif 'shoulder' in name:
                # Arms for counter-balancing
                correction = 0.2 * self.imu_data['orientation'].y
                commands.append(correction)
            elif 'elbow' in name:
                # Elbows bent for stability
                commands.append(0.8)
            elif 'neck' in name:
                # Keep neck stable
                commands.append(0.0)
            else:
                commands.append(0.0)
        
        return commands

    def walk_pattern(self):
        """Generate walking pattern commands."""
        t = self.get_clock().now().nanoseconds / 1e9  # Time in seconds
        
        commands = []
        for i, name in enumerate(self.joint_names):
            if 'hip' in name:
                # Hip joints move in opposition for walking
                if 'left' in name:
                    cmd = 0.2 * math.sin(t * 2.0)
                else:  # right
                    cmd = 0.2 * math.sin(t * 2.0 + math.pi)
            elif 'knee' in name:
                # Knees follow hips with phase offset
                if 'left' in name:
                    cmd = 0.15 * math.sin(t * 2.0 + math.pi/3)
                else:  # right
                    cmd = 0.15 * math.sin(t * 2.0 + math.pi + math.pi/3)
            elif 'ankle' in name:
                # Ankles for balance during walking
                cmd = -0.1 * self.imu_data['orientation'].y
            elif 'shoulder' in name:
                # Arms swing opposite to legs
                if 'left' in name:
                    cmd = 0.3 * math.sin(t * 2.0 + math.pi)
                else:  # right
                    cmd = 0.3 * math.sin(t * 2.0)
            elif 'elbow' in name:
                # Elbows bent during walking
                cmd = 0.6 + 0.2 * math.sin(t * 2.0)
            elif 'neck' in name:
                # Keep head steady
                cmd = 0.1 * math.sin(t * 0.5)
            else:
                cmd = 0.0
            
            commands.append(cmd)
        
        return commands

    def gesture_pattern(self):
        """Generate gesture pattern commands."""
        t = self.get_clock().now().nanoseconds / 1e9  # Time in seconds
        
        commands = []
        for i, name in enumerate(self.joint_names):
            if 'shoulder' in name:
                if 'left' in name:
                    # Left arm waves
                    cmd = 0.5 + 0.4 * math.sin(t * 4.0)
                else:  # right
                    # Right arm stays stable
                    cmd = 0.2
            elif 'elbow' in name:
                if 'left' in name:
                    # Left elbow bends for waving
                    cmd = 1.2 + 0.5 * math.sin(t * 4.0 + math.pi/2)
                else:  # right
                    cmd = 0.8
            elif 'hip' in name:
                # Keep hips stable during gesture
                cmd = 0.05 * math.sin(t * 0.5)
            elif 'knee' in name:
                cmd = 0.05 * math.sin(t * 0.5)
            elif 'ankle' in name:
                cmd = -0.05 * math.sin(t * 0.5)
            elif 'neck' in name:
                # Gentle head movement
                cmd = 0.2 * math.sin(t * 0.3)
            else:
                cmd = 0.0
            
            commands.append(cmd)
        
        return commands

    def cautious_pattern(self):
        """Generate cautious/balancing pattern."""
        commands = []
        for name in self.joint_names:
            if 'hip' in name:
                # Slight adjustment based on balance
                correction = -1.0 * self.imu_data['orientation'].y
                commands.append(correction)
            elif 'knee' in name:
                # Slightly bent for stability
                commands.append(0.1)
            elif 'ankle' in name:
                # Active balance correction
                correction = -0.8 * self.imu_data['orientation'].y
                commands.append(correction)
            elif 'shoulder' in name:
                # Arms out slightly for balance
                commands.append(0.3)
            elif 'elbow' in name:
                # Elbows bent
                commands.append(1.0)
            elif 'neck' in name:
                # Keep head steady
                commands.append(0.0)
            else:
                commands.append(0.0)
        
        return commands

    def idle_position(self):
        """Return to neutral position."""
        commands = []
        for name in self.joint_names:
            if 'hip' in name:
                commands.append(0.0)
            elif 'knee' in name:
                commands.append(0.0)
            elif 'ankle' in name:
                commands.append(0.0)
            elif 'shoulder' in name:
                commands.append(0.0)
            elif 'elbow' in name:
                commands.append(0.5)  # Slightly bent
            elif 'neck' in name:
                commands.append(0.0)
            else:
                commands.append(0.0)
        
        return commands


def main(args=None):
    """Main function for the complete nervous system."""
    rclpy.init(args=args)
    
    nervous_system = HumanoidNervousSystem()
    
    try:
        rclpy.spin(nervous_system)
    except KeyboardInterrupt:
        nervous_system.get_logger().info("Nervous system stopped by user")
    finally:
        nervous_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()