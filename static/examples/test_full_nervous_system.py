#!/usr/bin/env python3

"""
End-to-end test for the complete nervous system example.

This script validates that the full nervous system example works correctly,
testing all components and their integration for the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, Imu, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from threading import Thread
import time


class NervousSystemValidator(Node):
    """Validator node that monitors the nervous system and validates its operation."""

    def __init__(self):
        """Initialize the validator node."""
        super().__init__('nervous_system_validator')
        
        # Track all message types
        self.status_messages = []
        self.joint_commands = []
        self.sensor_data = []
        self.imu_data = []
        self.scan_data = []
        self.odom_data = []
        self.motor_commands = []
        
        # Subscriptions to monitor the nervous system
        self.status_subscription = self.create_subscription(
            String, '/nervous_system/status', self.status_callback, 10)
        self.joint_cmd_subscription = self.create_subscription(
            JointState, '/nervous_system/joint_commands', self.joint_cmd_callback, 10)
        self.sensor_subscription = self.create_subscription(
            JointState, '/nervous_system/sensor_data', self.sensor_callback, 10)
        self.imu_subscription = self.create_subscription(
            Imu, '/nervous_system/imu_data', self.imu_callback, 10)
        self.scan_subscription = self.create_subscription(
            LaserScan, '/nervous_system/scan_data', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, '/nervous_system/odometry', self.odom_callback, 10)
        self.motor_subscription = self.create_subscription(
            Twist, '/nervous_system/motor_commands', self.motor_callback, 10)
        
        # Service client for emergency stop
        self.emergency_stop_client = self.create_client(SetBool, '/nervous_system/emergency_stop')
        
        # Validation parameters
        self.validation_start_time = self.get_clock().now().nanoseconds / 1e9
        self.test_duration = 10.0  # seconds
        self.test_completed = False
        self.test_success = False

    def status_callback(self, msg):
        """Handle status messages."""
        self.status_messages.append(msg.data)
        self.get_logger().info(f'Validation: Received status - {msg.data}')

    def joint_cmd_callback(self, msg):
        """Handle joint command messages."""
        self.joint_commands.append(msg)
        self.get_logger().info(f'Validation: Received joint command for {len(msg.name)} joints')

    def sensor_callback(self, msg):
        """Handle sensor data messages."""
        self.sensor_data.append(msg)
        self.get_logger().info(f'Validation: Received sensor data for {len(msg.name)} joints')

    def imu_callback(self, msg):
        """Handle IMU data messages."""
        self.imu_data.append(msg)
        self.get_logger().info('Validation: Received IMU data')

    def scan_callback(self, msg):
        """Handle LIDAR scan messages."""
        self.scan_data.append(msg)
        self.get_logger().info(f'Validation: Received LIDAR scan with {len(msg.ranges)} ranges')

    def odom_callback(self, msg):
        """Handle odometry messages."""
        self.odom_data.append(msg)
        self.get_logger().info(f'Validation: Received odometry with pos=({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})')

    def motor_callback(self, msg):
        """Handle motor command messages."""
        self.motor_commands.append(msg)
        self.get_logger().info(f'Validation: Received motor command: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')

    def run_validation(self):
        """Run the validation test."""
        self.get_logger().info('Starting nervous system validation...')
        
        # Wait for service to be available
        while not self.emergency_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for emergency stop service...')
        
        # Wait for test duration
        start_time = self.get_clock().now().nanoseconds / 1e9
        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < self.test_duration:
            time.sleep(0.1)
        
        # Perform validation checks
        self.test_completed = True
        self.test_success = self.validate_results()
        
        # Send emergency stop to halt the system
        future = self.emergency_stop_client.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(self, future)
        
        self.print_validation_results()

    def validate_results(self):
        """Validate that all components are working correctly."""
        # Check that we received messages of each type
        checks = {
            'Status messages': len(self.status_messages) > 0,
            'Joint commands': len(self.joint_commands) > 0,
            'Sensor data': len(self.sensor_data) > 0,
            'IMU data': len(self.imu_data) > 0,
            'LIDAR scans': len(self.scan_data) > 0,
            'Odometry data': len(self.odom_data) > 0,
            'Motor commands': len(self.motor_commands) > 0
        }
        
        # Additional checks
        if checks['Joint commands']:
            # Check that we have the expected number of joints
            if len(self.joint_commands) > 0:
                latest_cmd = self.joint_commands[-1]
                checks['Correct joint count'] = len(latest_cmd.name) >= 10  # At least 10 joints
        
        if checks['LIDAR scans']:
            # Check that we have reasonable scan data
            if len(self.scan_data) > 0:
                latest_scan = self.scan_data[-1]
                checks['Valid scan range'] = len(latest_scan.ranges) > 0 and all(r > 0 for r in latest_scan.ranges if not float('inf') == r)
        
        # Overall validation
        all_checks_passed = all(checks.values())
        
        # Store detailed results
        self.validation_results = checks
        self.all_checks_passed = all_checks_passed
        
        return all_checks_passed

    def print_validation_results(self):
        """Print detailed validation results."""
        print("\n" + "=" * 60)
        print("NERVOUS SYSTEM VALIDATION RESULTS")
        print("=" * 60)
        
        for test_name, result in self.validation_results.items():
            status = "✅ PASS" if result else "❌ FAIL"
            print(f"{test_name:<30} {status}")
        
        print("-" * 60)
        overall_status = "✅ VALIDATION PASSED" if self.all_checks_passed else "❌ VALIDATION FAILED"
        print(f"{'Overall Result':<30} {overall_status}")
        print(f"{'Messages Received':<30} Status:{len(self.status_messages)}, Joints:{len(self.joint_commands)}, Sensors:{len(self.sensor_data)}")
        print("=" * 60)


def send_brain_commands():
    """Send commands to the nervous system brain interface."""
    rclpy.init()
    
    node = rclpy.create_node('brain_commander')
    brain_cmd_publisher = node.create_publisher(String, '/nervous_system/brain_commands', 10)
    
    # Send a sequence of commands
    time.sleep(2)  # Wait for connections
    
    commands = [
        'forward',
        'left',
        'stop',
        'right',
        'forward'
    ]
    
    for cmd in commands:
        msg = String()
        msg.data = cmd
        brain_cmd_publisher.publish(msg)
        node.get_logger().info(f'Sent brain command: {cmd}')
        time.sleep(2)
    
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    """Main function to run the nervous system validation."""
    print("Starting Nervous System End-to-End Validation")
    print("=" * 50)
    
    # Initialize ROS 2
    rclpy.init()
    
    # Create validator node
    validator = NervousSystemValidator()
    
    # Start brain commander in a separate thread
    brain_thread = Thread(target=send_brain_commands)
    brain_thread.start()
    
    # Run validation in the validator node
    def run_validation():
        validator.run_validation()
    
    validation_thread = Thread(target=run_validation)
    validation_thread.start()
    
    # Run the validator node to process messages
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        # Wait for threads to complete
        validation_thread.join()
        brain_thread.join()
        
        # Cleanup
        validator.destroy_node()
        rclpy.shutdown()
        
        return 0 if validator.test_success else 1


if __name__ == '__main__':
    main()