#!/usr/bin/env python3

"""
End-to-end test for the complete ROS 2 nervous system example.

This test verifies that the complete nervous system example works correctly
by simulating sensor inputs and verifying appropriate responses.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
import time
import threading


class NervousSystemTester(Node):
    """
    End-to-end test for the complete nervous system.
    """
    
    def __init__(self):
        super().__init__('nervous_system_tester')
        
        # Publishers to simulate sensor data
        self.joint_state_publisher = self.create_publisher(
            JointState, 'humanoid_sensors/joint_states', 10)
        
        self.imu_publisher = self.create_publisher(
            Imu, 'humanoid_sensors/imu', 10)
        
        # Subscribers to verify system responses
        self.joint_command_subscriber = self.create_subscription(
            Float64MultiArray, 'joint_commands', 
            self.joint_command_callback, 10)
        
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', 
            self.cmd_vel_callback, 10)
        
        self.status_subscriber = self.create_subscription(
            String, 'robot_status', 
            self.status_callback, 10)
        
        # Test tracking
        self.command_count = 0
        self.velocity_count = 0
        self.status_count = 0
        self.test_results = {
            'commands_received': False,
            'velocity_commands': False,
            'status_updates': False,
            'balance_response': False,
            'state_transitions': False
        }
        
        # Timer to run test scenarios
        self.test_timer = self.create_timer(0.1, self.run_test_scenario)
        self.test_phase = 0
        self.test_start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info("Nervous System Tester initialized")

    def joint_command_callback(self, msg):
        self.command_count += 1
        if self.command_count == 1:
            self.get_logger().info(f"✓ First joint command received with {len(msg.data)} values")
        if len(msg.data) > 0:
            self.test_results['commands_received'] = True

    def cmd_vel_callback(self, msg):
        self.velocity_count += 1
        if self.velocity_count == 1:
            self.get_logger().info("✓ First velocity command received")
        if abs(msg.linear.x) > 0 or abs(msg.angular.z) > 0:
            self.test_results['velocity_commands'] = True

    def status_callback(self, msg):
        self.status_count += 1
        if self.status_count == 1:
            self.get_logger().info(f"✓ First status update: {msg.data}")
        if 'State:' in msg.data:
            self.test_results['status_updates'] = True

    def run_test_scenario(self):
        """Run different test scenarios by publishing different sensor inputs."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.test_start_time
        
        if self.test_phase == 0:
            # Initial test: publish normal sensor data
            if elapsed > 1.0:  # Run for 1 second
                self.test_phase = 1
                self.get_logger().info("Phase 1: Testing balance response")
        
        if self.test_phase == 1:
            # Test balance response by publishing IMU with tilt
            self.publish_tilted_imu()
            if elapsed > 3.0:  # Run for additional 2 seconds
                self.test_phase = 2
                self.publish_normal_sensors()  # Reset to normal
                self.get_logger().info("Phase 2: Testing walking state")
        
        if self.test_phase == 2:
            # Test walking by creating conditions that should trigger walking
            self.publish_walking_conditions()
            if elapsed > 5.0:  # Run for additional 2 seconds
                self.test_phase = 3
                self.get_logger().info("Test scenario complete")
        
        # Publish joint states regularly
        self.publish_joint_states()
        
        # Check if balance response was detected
        if elapsed > 2.0 and elapsed < 3.0 and abs(self.last_imu_tilt) > 0.2:
            if self.command_count > 10:  # Should have received multiple commands
                self.test_results['balance_response'] = True
                self.get_logger().info("✓ Balance response detected")
    
    def publish_joint_states(self):
        """Publish normal joint state data."""
        msg = JointState()
        msg.name = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'neck_joint'
        ]
        # Neutral positions
        msg.position = [0.0] * len(msg.name)
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        self.joint_state_publisher.publish(msg)
    
    def publish_tilted_imu(self):
        """Publish IMU data with intentional tilt."""
        msg = Imu()
        
        # Create a tilt around Y-axis (pitch)
        tilt_angle = 0.3  # 0.3 radians ≈ 17 degrees
        msg.orientation.x = 0.0
        msg.orientation.y = tilt_angle
        msg.orientation.z = 0.0
        msg.orientation.w = (1.0 - tilt_angle**2)**0.5  # Normalize
        
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # Gravity
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        self.last_imu_tilt = tilt_angle
        self.imu_publisher.publish(msg)
    
    def publish_normal_sensors(self):
        """Publish normal sensor values."""
        self.publish_joint_states()  # Already doing this in timer
        
        # Publish normal IMU (upright)
        msg = Imu()
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        self.imu_publisher.publish(msg)
    
    def publish_walking_conditions(self):
        """Publish sensor data that should trigger walking state."""
        self.publish_joint_states()
        
        # Normal IMU (upright)
        msg = Imu()
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0  # No tilt
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        self.imu_publisher.publish(msg)

    def evaluate_test(self):
        """Evaluate overall test results."""
        self.get_logger().info("\n=== TEST RESULTS ===")
        
        passed_count = 0
        total_count = len(self.test_results)
        
        for test_name, result in self.test_results.items():
            status = "✓ PASS" if result else "✗ FAIL"
            self.get_logger().info(f"{test_name}: {status}")
            if result:
                passed_count += 1
        
        self.get_logger().info(f"\nOverall: {passed_count}/{total_count} tests passed")
        
        if passed_count == total_count:
            self.get_logger().info("🎉 All tests passed! The complete nervous system is working correctly.")
            return True
        else:
            self.get_logger().info("❌ Some tests failed. Check the implementation.")
            return False


def main(args=None):
    """Main test function."""
    rclpy.init(args=args)
    
    tester = NervousSystemTester()
    
    # Run the test for a specific duration
    test_duration = 6  # seconds
    start_time = time.time()
    
    try:
        # Spin with a timeout
        while time.time() - start_time < test_duration:
            rclpy.spin_once(tester, timeout_sec=0.1)
    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")
    finally:
        results = tester.evaluate_test()
        tester.destroy_node()
        rclpy.shutdown()
        
        # Exit with appropriate code
        return 0 if results else 1


if __name__ == '__main__':
    exit(main())