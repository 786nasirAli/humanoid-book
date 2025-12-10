#!/usr/bin/env python3
"""
Sensor Publisher Node

This node simulates realistic sensor data for LiDAR, Depth Camera, and IMU
with appropriate noise models and realistic characteristics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo, Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import numpy as np
import math
import time
from collections import deque


class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # Create publishers for each sensor
        self.lidar_pub = self.create_publisher(LaserScan, '/lidar/scan', 10)
        self.camera_pub = self.create_publisher(Image, '/depth_cam/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/depth_cam/camera_info', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Timer for sensor data publishing
        self.lidar_timer = self.create_timer(0.1, self.publish_lidar_data)  # 10Hz
        self.camera_timer = self.create_timer(1.0/30, self.publish_camera_data)  # 30Hz
        self.imu_timer = self.create_timer(0.01, self.publish_imu_data)  # 100Hz
        
        # Sensor parameters
        self.lidar_params = {
            'angle_min': -math.pi/2,
            'angle_max': math.pi/2,
            'angle_increment': math.pi / 720,  # 720 samples for 180 degrees
            'range_min': 0.1,
            'range_max': 30.0,
            'noise_stddev': 0.01  # 1cm noise
        }
        
        self.camera_params = {
            'width': 640,
            'height': 480,
            'fov': 60.0,  # degrees
            'noise_stddev': 0.007  # depth noise
        }
        
        self.imu_params = {
            'gyro_noise': 0.0017,  # rad/s
            'accel_noise': 0.017,   # m/s^2
            'gyro_bias_drift': 0.0001,
            'accel_bias_drift': 0.001
        }
        
        # Internal state
        self.lidar_angle = 0.0
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.accel_bias = [0.0, 0.0, 0.0]
        self.sequence_counter = 0
        
        self.get_logger().info('Sensor Publisher Node initialized')
    
    def add_noise(self, value, noise_stddev, bias=0.0):
        """Add Gaussian noise to sensor data"""
        noise = np.random.normal(0.0, noise_stddev)
        return value + noise + bias
    
    def publish_lidar_data(self):
        """Publish realistic LiDAR data with noise"""
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.header.seq = self.sequence_counter
        
        # Set laser scan parameters
        msg.angle_min = self.lidar_params['angle_min']
        msg.angle_max = self.lidar_params['angle_max']
        msg.angle_increment = self.lidar_params['angle_increment']
        msg.time_increment = 0.0  # Not used for simulation
        msg.scan_time = 0.1  # 10Hz
        msg.range_min = self.lidar_params['range_min']
        msg.range_max = self.lidar_params['range_max']
        
        # Generate simulated ranges with some pattern to make it realistic
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        ranges = []
        
        for i in range(num_readings):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Simulate an environment with some objects at specific distances
            if abs(angle) < 0.2:  # Front of robot
                true_range = 2.0  # Wall 2 meters away
            elif abs(angle - 0.5) < 0.1 or abs(angle + 0.5) < 0.1:  # Sides
                true_range = 1.5  # Objects at 1.5m
            else:
                true_range = 5.0  # Far objects
            
            # Add realistic noise
            noisy_range = self.add_noise(
                true_range, 
                self.lidar_params['noise_stddev']
            )
            
            # Ensure range is within valid bounds
            if noisy_range < msg.range_min:
                noisy_range = float('inf')  # Invalid reading
            elif noisy_range > msg.range_max:
                noisy_range = float('inf')  # Invalid reading
                
            ranges.append(noisy_range)
        
        msg.ranges = ranges
        msg.intensities = []  # Not implemented
        
        self.lidar_pub.publish(msg)
        self.sequence_counter += 1
        
        self.get_logger().debug(f'Published LiDAR data with {len(ranges)} readings')
    
    def publish_camera_data(self):
        """Publish realistic depth camera data"""
        # Publish depth image
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_link'
        img_msg.header.seq = self.sequence_counter
        
        img_msg.height = self.camera_params['height']
        img_msg.width = self.camera_params['width']
        img_msg.encoding = '32FC1'  # 32-bit float, single channel (depth)
        img_msg.is_bigendian = 0
        img_msg.step = self.camera_params['width'] * 4  # 4 bytes per pixel (float32)
        
        # Generate simulated depth image data
        # Create a pattern: closer in center, farther on edges
        data = []
        center_x, center_y = img_msg.width / 2, img_msg.height / 2
        
        for y in range(img_msg.height):
            for x in range(img_msg.width):
                # Calculate distance from center (normalized)
                dist_from_center = math.sqrt((x - center_x)**2 + (y - center_y)**2)
                max_dist = math.sqrt(center_x**2 + center_y**2)
                normalized_dist = dist_from_center / max_dist
                
                # Simulate depth: closer in center, farther on edges
                true_depth = 1.0 + 4.0 * normalized_dist  # 1m to 5m
                
                # Add realistic noise based on distance from optical center
                pixel_noise = self.camera_params['noise_stddev'] * (1 + normalized_dist)
                noisy_depth = self.add_noise(true_depth, pixel_noise)
                
                # Ensure depth is positive and reasonable
                if noisy_depth < 0.1:
                    noisy_depth = float('inf')
                elif noisy_depth > 10.0:
                    noisy_depth = float('inf')
                    
                data.append(noisy_depth)
        
        # Convert to byte array
        import struct
        img_msg.data = b''.join(struct.pack('f', val) for val in data)
        
        self.camera_pub.publish(img_msg)
        
        # Publish camera info
        info_msg = CameraInfo()
        info_msg.header = img_msg.header
        info_msg.width = img_msg.width
        info_msg.height = img_msg.height
        
        # Calculate focal length from FOV
        fov_rad = math.radians(self.camera_params['fov'])
        focal_length = (img_msg.width / 2) / math.tan(fov_rad / 2)
        
        info_msg.k = [focal_length, 0.0, img_msg.width/2,
                     0.0, focal_length, img_msg.height/2,
                     0.0, 0.0, 1.0]
        
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.p = [focal_length, 0.0, img_msg.width/2, 0.0,
                     0.0, focal_length, img_msg.height/2, 0.0,
                     0.0, 0.0, 1.0, 0.0]
        
        info_msg.binning_x = 0
        info_msg.binning_y = 0
        info_msg.roi.x_offset = 0
        info_msg.roi.y_offset = 0
        info_msg.roi.height = 0
        info_msg.roi.width = 0
        info_msg.roi.do_rectify = False
        
        self.camera_info_pub.publish(info_msg)
        
        self.get_logger().debug(f'Published depth camera data {img_msg.width}x{img_msg.height}')
    
    def publish_imu_data(self):
        """Publish realistic IMU data with noise and bias drift"""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.header.seq = self.sequence_counter
        
        # For simulation purposes, assume the robot is mostly stationary
        # but with small movements
        current_time = time.time()
        
        # Simulate small oscillations
        gyro_x = 0.01 * math.sin(0.5 * current_time)  # Small angular velocity
        gyro_y = 0.01 * math.cos(0.3 * current_time)
        gyro_z = 0.005 * math.sin(0.7 * current_time)
        
        # Linear acceleration (simulating gravity + small movements)
        accel_x = 0.1 * math.sin(0.2 * current_time)
        accel_y = 0.1 * math.cos(0.3 * current_time)
        accel_z = 9.81  # Gravity
        
        # Add noise to gyro readings
        msg.angular_velocity = Vector3()
        msg.angular_velocity.x = self.add_noise(
            gyro_x, 
            self.imu_params['gyro_noise'],
            self.gyro_bias[0]
        )
        msg.angular_velocity.y = self.add_noise(
            gyro_y, 
            self.imu_params['gyro_noise'],
            self.gyro_bias[1]
        )
        msg.angular_velocity.z = self.add_noise(
            gyro_z, 
            self.imu_params['gyro_noise'],
            self.gyro_bias[2]
        )
        
        # Add noise to acceleration readings
        msg.linear_acceleration = Vector3()
        msg.linear_acceleration.x = self.add_noise(
            accel_x, 
            self.imu_params['accel_noise'],
            self.accel_bias[0]
        )
        msg.linear_acceleration.y = self.add_noise(
            accel_y, 
            self.imu_params['accel_noise'],
            self.accel_bias[1]
        )
        msg.linear_acceleration.z = self.add_noise(
            accel_z, 
            self.imu_params['accel_noise'],
            self.accel_bias[2]
        )
        
        # Orientation (for this simulation, assume no significant orientation change)
        msg.orientation = Quaternion()
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        
        # Add covariance values (indicates uncertainty)
        # For a real IMU, these would be determined by calibration
        identity_3x3 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.angular_velocity_covariance = identity_3x3[:]
        msg.linear_acceleration_covariance = identity_3x3[:]
        msg.orientation_covariance = identity_3x3[:]
        
        # Update bias drifts slowly
        self.gyro_bias[0] += np.random.normal(0.0, self.imu_params['gyro_bias_drift'])
        self.gyro_bias[1] += np.random.normal(0.0, self.imu_params['gyro_bias_drift'])
        self.gyro_bias[2] += np.random.normal(0.0, self.imu_params['gyro_bias_drift'])
        
        self.accel_bias[0] += np.random.normal(0.0, self.imu_params['accel_bias_drift'])
        self.accel_bias[1] += np.random.normal(0.0, self.imu_params['accel_bias_drift'])
        self.accel_bias[2] += np.random.normal(0.0, self.imu_params['accel_bias_drift'])
        
        self.imu_pub.publish(msg)
        
        self.get_logger().debug(f'Published IMU data - Gyro: ({msg.angular_velocity.x:.4f}, {msg.angular_velocity.y:.4f}, {msg.angular_velocity.z:.4f})')


def main(args=None):
    rclpy.init(args=args)
    
    sensor_publisher = SensorPublisher()
    
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        sensor_publisher.get_logger().info('Shutting down Sensor Publisher Node...')
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()