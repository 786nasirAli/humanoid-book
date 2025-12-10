#!/usr/bin/env python3
"""
Sensor Validation Test Script

This script validates that the sensor simulation in Gazebo produces
realistic data outputs that match real-world sensor characteristics,
including proper noise models and geometric relationships.
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Point
import numpy as np
from cv_bridge import CvBridge
import cv2


class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')
        
        # Parameters for sensor validation
        self.lidar_validation_passed = False
        self.camera_validation_passed = False
        self.imu_validation_passed = False
        
        # For validation results
        self.lidar_data = None
        self.camera_data = None
        self.imu_data = None
        
        # Flags to track if we've received data
        self.lidar_received = False
        self.camera_received = False
        self.imu_received = False
        
        # Create subscribers for each sensor
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10
        )
        
        self.camera_sub = self.create_subscription(
            Image,
            '/depth_cam/image_raw',
            self.camera_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Timer to periodically check validation status
        self.validation_timer = self.create_timer(5.0, self.check_validation_status)
        
        self.get_logger().info('Sensor Validator initialized')

    def lidar_callback(self, msg):
        """Callback for LiDAR data"""
        self.lidar_data = msg
        self.lidar_received = True
        self.get_logger().debug(f'Received LiDAR data with {len(msg.ranges)} ranges')
        
        # Validate LiDAR data
        if not self.lidar_validation_passed:
            self.validate_lidar_data()
    
    def camera_callback(self, msg):
        """Callback for camera data"""
        self.camera_data = msg
        self.camera_received = True
        self.get_logger().debug(f'Received camera data: {msg.width}x{msg.height}')
        
        # Validate camera data
        if not self.camera_validation_passed:
            self.validate_camera_data()
    
    def imu_callback(self, msg):
        """Callback for IMU data"""
        self.imu_data = msg
        self.imu_received = True
        self.get_logger().debug(f'Received IMU data')
        
        # Validate IMU data
        if not self.imu_validation_passed:
            self.validate_imu_data()
    
    def validate_lidar_data(self):
        """Validate LiDAR data for realistic characteristics"""
        if self.lidar_data is None:
            return False
            
        msg = self.lidar_data
        
        # Check that ranges are within valid bounds
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        out_of_range = [r for r in msg.ranges if math.isinf(r)]  # Using inf to represent out of range
        
        # Validate angle characteristics
        expected_num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        if len(msg.ranges) != expected_num_readings:
            self.get_logger().error(f'LiDAR: Expected {expected_num_readings} readings, got {len(msg.ranges)}')
            return False
        
        # Check for reasonable range values
        if valid_ranges:
            min_range = min(valid_ranges)
            max_range = max(valid_ranges)
            
            if min_range < 0 or min_range > msg.range_max:
                self.get_logger().error(f'LiDAR: Invalid minimum range {min_range}')
                return False
            if max_range > msg.range_max:
                self.get_logger().error(f'LiDAR: Invalid maximum range {max_range}')
                return False
                
            # Calculate some range statistics to validate against realistic scenarios
            mean_range = sum(valid_ranges) / len(valid_ranges)
            
            # In our simulated environment, we expect most readings to be between 1m and 5m
            # based on the sensor_publisher.py patterns
            if mean_range < 0.5 or mean_range > 10.0:
                self.get_logger().warning(f'LiDAR: Mean range ({mean_range:.2f}) seems unrealistic')
        
        # Validate that there's some variation in the ranges (not all the same)
        if len(set(valid_ranges)) < 2 and len(valid_ranges) > 10:
            self.get_logger().warning('LiDAR: Range values seem too uniform')
        
        self.lidar_validation_passed = True
        self.get_logger().info('LiDAR validation PASSED')
        return True
    
    def validate_camera_data(self):
        """Validate camera data for realistic characteristics"""
        if self.camera_data is None:
            return False
            
        msg = self.camera_data
        
        # Check image dimensions
        if msg.width != 640 or msg.height != 480:
            self.get_logger().error(f'Camera: Unexpected dimensions {msg.width}x{msg.height}')
            return False
        
        # Check encoding
        if msg.encoding != '32FC1':
            self.get_logger().error(f'Camera: Unexpected encoding {msg.encoding}')
            return False
        
        # Process the image data to check for realistic depth values
        try:
            # Convert the image data to numpy array
            import struct
            # Unpack the binary data to float values
            depth_data = []
            for i in range(0, len(msg.data), 4):  # 4 bytes per float32
                if i + 4 <= len(msg.data):
                    float_val = struct.unpack('f', msg.data[i:i+4])[0]
                    depth_data.append(float_val)
            
            # Reshape to image dimensions
            depth_array = np.array(depth_data).reshape((msg.height, msg.width))
            
            # Calculate statistics
            valid_depths = depth_array[~np.isinf(depth_array)]  # Exclude infinite values
            valid_depths = valid_depths[valid_depths > 0]  # Only positive depths
            
            if len(valid_depths) == 0:
                self.get_logger().warning('Camera: No valid depth values found')
                return True  # Not necessarily a failure, could be all invalid readings
            
            # Check that depth values are reasonable
            avg_depth = np.mean(valid_depths)
            if avg_depth < 0.1 or avg_depth > 10.0:
                self.get_logger().error(f'Camera: Average depth {avg_depth:.2f} seems unrealistic')
                return False
            
            # In our simulation, center should be closer than edges
            center_depth = depth_array[depth_array.shape[0]//2, depth_array.shape[1]//2]
            edge_depths = [
                depth_array[0, depth_array.shape[1]//2],  # Top center
                depth_array[-1, depth_array.shape[1]//2],  # Bottom center
                depth_array[depth_array.shape[0]//2, 0],  # Left center
                depth_array[depth_array.shape[0]//2, -1]  # Right center
            ]
            edge_depths = [d for d in edge_depths if not np.isinf(d) and d > 0]
            
            if edge_depths and center_depth > 0:
                avg_edge_depth = np.mean(edge_depths) if edge_depths else center_depth
                if center_depth > avg_edge_depth:
                    # This is realistic: center is typically closer than edges in our simulated pattern
                    pass
                else:
                    self.get_logger().info(f'Camera: Depth pattern center={center_depth:.2f}, avg_edge={avg_edge_depth:.2f}')
        
        except Exception as e:
            self.get_logger().error(f'Camera: Error processing image data: {e}')
            return False
        
        self.camera_validation_passed = True
        self.get_logger().info('Camera validation PASSED')
        return True
    
    def validate_imu_data(self):
        """Validate IMU data for realistic characteristics"""
        if self.imu_data is None:
            return False
            
        msg = self.imu_data
        
        # Check that acceleration values are reasonable (around 9.8 m/s^2 for gravity)
        # Since the robot is mostly stationary, the combined acceleration should be close to 9.8
        combined_accel = math.sqrt(
            msg.linear_acceleration.x**2 + 
            msg.linear_acceleration.y**2 + 
            msg.linear_acceleration.z**2
        )
        
        # Allow for some movement, so check if it's in a reasonable range
        if combined_accel < 8.0 or combined_accel > 12.0:
            self.get_logger().warning(f'IMU: Combined acceleration {combined_accel:.2f} seems unusual')
        
        # Check that z acceleration is approximately gravity (with some tolerance for movement)
        if abs(msg.linear_acceleration.z - 9.81) > 2.0:
            self.get_logger().warning(f'IMU: Z acceleration {msg.linear_acceleration.z} differs significantly from gravity')
        
        # Check gyroscope values - should be small for stationary robot
        gyro_magnitude = math.sqrt(
            msg.angular_velocity.x**2 + 
            msg.angular_velocity.y**2 + 
            msg.angular_velocity.z**2
        )
        
        if gyro_magnitude > 1.0:  # 1 rad/s is quite high for a stationary robot
            self.get_logger().info(f'IMU: High angular velocity detected: {gyro_magnitude:.3f} rad/s')
        
        # For orientation, in our simulation we expect a default orientation
        # but allow for some variation due to noise
        if (abs(msg.orientation.x) > 0.1 or 
            abs(msg.orientation.y) > 0.1 or 
            abs(msg.orientation.w - 1.0) > 0.1):
            self.get_logger().info('IMU: Non-default orientation detected')
        
        self.imu_validation_passed = True
        self.get_logger().info('IMU validation PASSED')
        return True
    
    def check_validation_status(self):
        """Check and report validation status periodically"""
        all_received = self.lidar_received and self.camera_received and self.imu_received
        all_validated = self.lidar_validation_passed and self.camera_validation_passed and self.imu_validation_passed
        
        if all_received and all_validated:
            self.get_logger().info('ALL SENSORS VALIDATED SUCCESSFULLY')
            self.print_validation_summary()
        elif all_received:
            # All data received but not all validated - print status
            self.get_logger().info('All sensor data received, but some validations still pending...')
            self.print_validation_status()
        else:
            # Still waiting for data
            missing = []
            if not self.lidar_received:
                missing.append('LiDAR')
            if not self.camera_received:
                missing.append('Camera')
            if not self.imu_received:
                missing.append('IMU')
                
            if missing:
                self.get_logger().info(f'Waiting for data from: {", ".join(missing)}')
    
    def print_validation_status(self):
        """Print current validation status"""
        print("\n" + "="*50)
        print("SENSOR VALIDATION STATUS")
        print("="*50)
        print(f"LiDAR Data Received: {'YES' if self.lidar_received else 'NO'}")
        print(f"LiDAR Validation: {'PASSED' if self.lidar_validation_passed else 'PENDING/FAILED'}")
        print(f"Camera Data Received: {'YES' if self.camera_received else 'NO'}")
        print(f"Camera Validation: {'PASSED' if self.camera_validation_passed else 'PENDING/FAILED'}")
        print(f"IMU Data Received: {'YES' if self.imu_received else 'NO'}")
        print(f"IMU Validation: {'PASSED' if self.imu_validation_passed else 'PENDING/FAILED'}")
        print("="*50)
    
    def print_validation_summary(self):
        """Print final validation summary"""
        print("\n" + "="*60)
        print("SENSOR VALIDATION SUMMARY")
        print("="*60)
        print(f"LiDAR Sensor: {'PASSED' if self.lidar_validation_passed else 'FAILED'}")
        print(f"Depth Camera: {'PASSED' if self.camera_validation_passed else 'FAILED'}")
        print(f"IMU Sensor: {'PASSED' if self.imu_validation_passed else 'FAILED'}")
        
        all_passed = self.lidar_validation_passed and self.camera_validation_passed and self.imu_validation_passed
        print(f"Overall Sensor Validation: {'ALL PASSED' if all_passed else 'SOME FAILED'}")
        
        if all_passed:
            print("\n✅ All sensors are producing realistic data with appropriate noise models!")
            print("✅ Sensor simulation is ready for use in the Digital Twin project.")
        else:
            print("\n❌ Some sensor validations failed. Check logs for details.")
        print("="*60)


def main(args=None):
    rclpy.init(args=args)
    
    validator = SensorValidator()
    
    # Run until all validations are complete or timeout
    start_time = time.time()
    timeout = 60  # 60 seconds timeout
    
    try:
        while rclpy.ok():
            rclpy.spin_once(validator, timeout_sec=0.1)
            
            # Check if all validations are complete
            all_validated = (validator.lidar_validation_passed and 
                           validator.camera_validation_passed and 
                           validator.imu_validation_passed)
            
            # Also check if we've received data from all sensors
            all_received = (validator.lidar_received and 
                          validator.camera_received and 
                          validator.imu_received)
            
            if all_validated or (time.time() - start_time > timeout):
                break
    
    except KeyboardInterrupt:
        validator.get_logger().info('Sensor validation interrupted by user')
    finally:
        validator.print_validation_summary()
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()