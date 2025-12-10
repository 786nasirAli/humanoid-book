#!/usr/bin/env python3

"""
Synchronization validation test for Gazebo-Unity environment
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import math


class SyncValidator(Node):
    def __init__(self):
        super().__init__('sync_validator')
        
        # Subscriptions for Gazebo and Unity states
        self.gazebo_odom_sub = self.create_subscription(
            Odometry, '/gazebo/odom', self.gazebo_odom_callback, 10)
            
        self.unity_odom_sub = self.create_subscription(
            Odometry, '/unity/odom', self.unity_odom_callback, 10)
            
        self.gazebo_joint_sub = self.create_subscription(
            JointState, '/gazebo/joint_states', self.gazebo_joint_callback, 10)
            
        self.unity_joint_sub = self.create_subscription(
            JointState, '/unity/joint_states', self.unity_joint_callback, 10)
        
        # Storage for states
        self.gazebo_odom = None
        self.unity_odom = None
        self.gazebo_joints = None
        self.unity_joints = None
        
        # Timer for validation
        self.timer = self.create_timer(1.0, self.validate_sync)
        self.validation_count = 0
        
        self.get_logger().info('Synchronization Validator Node Started')
    
    def gazebo_odom_callback(self, msg):
        self.gazebo_odom = msg
        
    def unity_odom_callback(self, msg):
        self.unity_odom = msg
        
    def gazebo_joint_callback(self, msg):
        self.gazebo_joints = msg
        
    def unity_joint_callback(self, msg):
        self.unity_joints = msg
    
    def validate_sync(self):
        """
        Validate synchronization accuracy between Gazebo and Unity
        """
        self.validation_count += 1
        self.get_logger().info(f'Running validation #{self.validation_count}')
        
        # Check odom synchronization
        if self.gazebo_odom and self.unity_odom:
            pos_diff = self.calculate_position_difference(
                self.gazebo_odom.pose.pose, 
                self.unity_odom.pose.pose
            )
            
            if pos_diff < 0.05:  # Within 5cm tolerance
                self.get_logger().info(
                    f'Odom sync validation passed - difference: {pos_diff:.3f}m')
            else:
                self.get_logger().warn(
                    f'Odom sync validation failed - difference: {pos_diff:.3f}m (threshold: 0.05m)')
        
        # Check joint state synchronization
        if self.gazebo_joints and self.unity_joints:
            joint_diff = self.calculate_joint_difference(
                self.gazebo_joints, 
                self.unity_joints
            )
            
            if joint_diff < 0.02:  # Within 0.02 rad tolerance
                self.get_logger().info(
                    f'Joint sync validation passed - difference: {joint_diff:.3f}rad')
            else:
                self.get_logger().warn(
                    f'Joint sync validation failed - difference: {joint_diff:.3f}rad (threshold: 0.02rad)')

    def calculate_position_difference(self, pose1, pose2):
        """
        Calculate Euclidean distance between two poses
        """
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def calculate_joint_difference(self, joints1, joints2):
        """
        Calculate average absolute difference between joint states
        """
        if len(joints1.position) != len(joints2.position):
            return float('inf')
        
        total_diff = 0.0
        for i in range(len(joints1.position)):
            total_diff += abs(joints1.position[i] - joints2.position[i])
        
        return total_diff / len(joints1.position)


def main(args=None):
    rclpy.init(args=args)
    
    validator = SyncValidator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()