#!/usr/bin/env python3

"""
Comprehensive example integrating all components of the ROS 2 nervous system.

This script demonstrates how all components of the ROS 2-based nervous system
work together in a humanoid robot, including nodes, topics, services, 
parameter management, and action execution.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, Imu, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_srvs.srv import SetBool
from builtin_interfaces.msg import Time
import math
import time
from threading import Thread


class NervousSystemMonitor(Node):
    """Main node that monitors and coordinates all nervous system components."""

    def __init__(self):
        """Initialize the nervous system monitor node."""
        super().__init__('nervous_system_monitor')
        
        # Create QoS profile for real-time communication
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Publishers for all system components
        self.joint_publisher = self.create_publisher(JointState, '/nervous_system/joint_commands', qos_profile)
        self.sensor_publisher = self.create_publisher(JointState, '/nervous_system/sensor_data', qos_profile)
        self.imu_publisher = self.create_publisher(Imu, '/nervous_system/imu_data', qos_profile)
        self.scan_publisher = self.create_publisher(LaserScan, '/nervous_system/scan_data', qos_profile)
        self.odom_publisher = self.create_publisher(Odometry, '/nervous_system/odometry', qos_profile)
        self.motor_cmd_publisher = self.create_publisher(Twist, '/nervous_system/motor_commands', qos_profile)
        self.status_publisher = self.create_publisher(String, '/nervous_system/status', 10)
        
        # Subscribers to monitor system state
        self.joint_subscription = self.create_subscription(
            JointState, '/nervous_system/joint_states', self.joint_state_callback, qos_profile)
        self.brain_subscription = self.create_subscription(
            String, '/nervous_system/brain_commands', self.brain_command_callback, 10)
        
        # Service server for nervous system operations
        self.emergency_stop_service = self.create_service(
            SetBool, '/nervous_system/emergency_stop', self.emergency_stop_callback)
        
        # Timer for system updates
        timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(timer_period, self.system_update)
        
        # System state tracking
        self.system_active = True
        self.time_counter = 0
        self.brain_commands = []
        self.joint_states = {}
        
        # Define humanoid joint names
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'head_yaw_joint', 'head_pitch_joint'
        ]

        self.get_logger().info('Nervous System Monitor initialized')

    def joint_state_callback(self, msg):
        """Update joint states from feedback."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_states[name] = msg.position[i]
        
        self.get_logger().info(f'Updated joint states for {len(msg.name)} joints')

    def brain_command_callback(self, msg):
        """Receive commands from the 'brain' of the robot."""
        self.brain_commands.append(msg.data)
        self.get_logger().info(f'Received brain command: {msg.data}')

    def emergency_stop_callback(self, request, response):
        """Handle emergency stop requests."""
        self.system_active = not request.data
        response.success = True
        response.message = f'Emergency stop {"activated" if self.system_active else "deactivated"}'
        self.get_logger().info(response.message)
        return response

    def system_update(self):
        """Main system update loop."""
        if not self.system_active:
            self.get_logger().warn('System is inactive due to emergency stop')
            return

        # Publish system status
        status_msg = String()
        status_msg.data = f'Nervous system active, cycle: {self.time_counter}'
        self.status_publisher.publish(status_msg)
        
        # Publish joint commands with coordinated patterns
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.header.frame_id = 'base_link'
        joint_cmd.name = self.joint_names
        
        positions = []
        for i, name in enumerate(self.joint_names):
            # Create different movement patterns for different joint types
            if 'hip' in name or 'knee' in name or 'ankle' in name:
                # Leg joints - walking pattern
                pos = math.sin(self.time_counter / 10.0 + i) * 0.3
            elif 'shoulder' in name or 'elbow' in name:
                # Arm joints - reaching pattern
                pos = math.sin(self.time_counter / 15.0 + i*0.5) * 0.4
            else:
                # Head joints - scanning pattern
                pos = math.sin(self.time_counter / 20.0 + i*0.7) * 0.2
            
            positions.append(pos)
        
        joint_cmd.position = positions
        joint_cmd.velocity = [0.0] * len(positions)
        joint_cmd.effort = [0.0] * len(positions)
        
        self.joint_publisher.publish(joint_cmd)
        
        # Publish simulated sensor data
        self.publish_sensor_data()
        
        # Publish simulated odometry
        self.publish_odometry()
        
        # Publish motor commands
        self.publish_motor_commands()
        
        self.time_counter += 1

    def publish_sensor_data(self):
        """Publish simulated sensor data."""
        # Joint sensor data
        sensor_data = JointState()
        sensor_data.header.stamp = self.get_clock().now().to_msg()
        sensor_data.header.frame_id = 'sensor_frame'
        sensor_data.name = self.joint_names
        sensor_data.position = [math.sin(self.time_counter/20.0 + i)*0.1 for i in range(len(self.joint_names))]
        sensor_data.velocity = [math.cos(self.time_counter/20.0 + i)*0.1 for i in range(len(self.joint_names))]
        sensor_data.effort = [0.0] * len(self.joint_names)
        self.sensor_publisher.publish(sensor_data)
        
        # IMU data
        imu_data = Imu()
        imu_data.header.stamp = self.get_clock().now().to_msg()
        imu_data.header.frame_id = 'imu_frame'
        imu_data.orientation.x = math.sin(self.time_counter/30.0) * 0.1
        imu_data.orientation.y = math.cos(self.time_counter/30.0) * 0.1
        imu_data.orientation.z = 0.0
        imu_data.orientation.w = 1.0
        self.imu_publisher.publish(imu_data)
        
        # LIDAR scan data
        scan_data = LaserScan()
        scan_data.header.stamp = self.get_clock().now().to_msg()
        scan_data.header.frame_id = 'lidar_frame'
        scan_data.angle_min = -math.pi/2
        scan_data.angle_max = math.pi/2
        scan_data.angle_increment = math.pi/180  # 1 degree
        scan_data.time_increment = 0.0
        scan_data.scan_time = 0.1
        scan_data.range_min = 0.1
        scan_data.range_max = 10.0
        scan_data.ranges = [5.0 + math.sin(self.time_counter/10.0 + i*math.pi/18)*2.0 for i in range(180)]
        self.scan_publisher.publish(scan_data)

    def publish_odometry(self):
        """Publish simulated odometry data."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Simulate movement
        odom.pose.pose.position.x = math.sin(self.time_counter/50.0) * 2.0
        odom.pose.pose.position.y = math.cos(self.time_counter/50.0) * 1.0
        odom.pose.pose.position.z = 0.0
        
        # Simple orientation
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.time_counter/100.0) * 0.5
        
        # Set velocity
        odom.twist.twist.linear.x = 0.2  # Constant forward velocity
        odom.twist.twist.angular.z = math.sin(self.time_counter/50.0) * 0.1  # Oscillating turn
        
        self.odom_publisher.publish(odom)

    def publish_motor_commands(self):
        """Publish motor commands based on brain input."""
        # If we have brain commands, try to follow them
        if self.brain_commands:
            cmd = self.brain_commands[-1]  # Use the latest command
            twist = Twist()
            
            if 'forward' in cmd:
                twist.linear.x = 0.3
            elif 'backward' in cmd:
                twist.linear.x = -0.3
            elif 'left' in cmd:
                twist.angular.z = 0.3
            elif 'right' in cmd:
                twist.angular.z = -0.3
            elif 'stop' in cmd:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                # Default movement pattern
                twist.linear.x = 0.1
                twist.angular.z = math.sin(self.time_counter/30.0) * 0.1
            
            self.motor_cmd_publisher.publish(twist)


def run_full_nervous_system():
    """Run the complete nervous system demonstration."""
    print("Starting Full Nervous System Integration Demo")
    print("=" * 50)
    
    # Initialize ROS 2
    rclpy.init()
    
    # Create the nervous system monitor
    monitor = NervousSystemMonitor()
    
    try:
        # Run the system
        print("Nervous system running... Press Ctrl+C to stop")
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nShutting down nervous system...")
    finally:
        # Cleanup
        monitor.destroy_node()
        rclpy.shutdown()
        
        print("Nervous system shutdown complete")
        return True


def main(args=None):
    """Main function to run the nervous system integration demo."""
    success = run_full_nervous_system()
    return 0 if success else 1


if __name__ == '__main__':
    main()