---
sidebar_position: 4
title: Sensor Emulation in Digital Twin
---

# Sensor Emulation: LiDAR, Depth Cameras, and IMUs

This section covers the simulation of various sensors in the digital twin environment, ensuring realistic outputs that match real-world sensor characteristics.

## LiDAR Simulation

LiDAR simulation provides 360-degree or 2D distance measurements:

### Configuration
- Sample count: 720 samples for 360° (0.5° resolution)
- Range: 0.1m to 30m
- Update rate: 10 Hz
- Noise model: Gaussian with 1cm standard deviation

### Example Configuration
```xml
<sensor name="lidar_2d" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>  <!-- -π/2 -->
        <max_angle>1.570796</max_angle>   <!-- π/2 -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <topic_name>/lidar/scan</topic_name>
  </plugin>
</sensor>
```

## Depth Camera Simulation

Depth cameras provide 3D information about the environment:

### Configuration
- Resolution: 640x480 pixels
- Field of view: 60 degrees
- Range: 0.1m to 10m
- Update rate: 30 Hz
- Noise characteristics: Distance-dependent noise

### Implementation Details
Depth cameras simulate realistic noise patterns based on:
- Distance from optical center
- Object distance (noise increases with square of distance)
- Ambient lighting conditions

## IMU Simulation

Inertial Measurement Units (IMUs) provide acceleration and angular velocity:

### Configuration
- Update rate: 100 Hz
- Angular velocity noise: ~0.1°/s standard deviation
- Linear acceleration noise: ~17mg standard deviation
- Bias drift characteristics

### Noise Modeling
IMUs incorporate realistic noise models including:
- Gyroscope white noise and bias drift
- Accelerometer white noise and bias drift
- Correlated noise between axes

## Sensor Integration with Robot Model

Sensors are mounted on the robot model:

```xml
<link name="lidar_link">
  <sensor name="lidar_2d">
    <!-- LiDAR configuration -->
  </sensor>
</link>

<link name="camera_link">
  <sensor name="depth_camera">
    <!-- Camera configuration -->
  </sensor>
</link>

<link name="imu_link">
  <sensor name="imu_sensor">
    <!-- IMU configuration -->
  </sensor>
</link>
```

## Validation of Sensor Outputs

Validate sensor outputs against geometric realities:

1. LiDAR measurements against known distances
2. Depth camera readings at various distances
3. IMU readings during controlled motion
4. Cross-validation between sensors

## Troubleshooting Sensor Issues

Common sensor simulation issues:
- Incorrect sensor mounting (check URDF/SDF)
- Inconsistent coordinate frames
- Unrealistic noise levels
- Timing synchronization problems
- Topic connectivity issues

## Practical Assignment - Sensor Emulation Implementation

### Assignment 1: Implementing Multiple Sensor Types on a Humanoid Robot

**Objective**: Add various sensor types to a humanoid robot model in Gazebo for comprehensive environment perception.

**Detailed Steps**:
1. **Create a Humanoid Robot Model with Sensors**:
   ```xml
   <!-- urdf/humanoid_with_sensors.urdf.xacro -->
   <?xml version="1.0" ?>
   <robot name="humanoid_with_sensors" xmlns:xacro="http://www.ros.org/wiki/xacro">
     <!-- Properties -->
     <xacro:property name="M_PI" value="3.14159"/>

     <!-- Main body -->
     <link name="base_link">
       <visual>
         <geometry>
           <box size="0.2 0.2 0.8"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 1 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.2 0.2 0.8"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="10.0"/>
         <origin xyz="0 0 0.4"/>
         <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.05"/>
       </inertial>
     </link>

     <!-- Head with cameras -->
     <joint name="neck_joint" type="revolute">
       <parent link="base_link"/>
       <child link="head"/>
       <origin xyz="0 0 0.8" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="${-M_PI/3}" upper="${M_PI/3}" effort="10" velocity="1"/>
     </joint>

     <link name="head">
       <visual>
         <geometry>
           <sphere radius="0.15"/>
         </geometry>
         <material name="white">
           <color rgba="1 1 1 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <sphere radius="0.15"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="2.0"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <!-- Left eye camera -->
     <joint name="left_camera_joint" type="fixed">
       <parent link="head"/>
       <child link="left_camera"/>
       <origin xyz="0.05 0.05 0" rpy="0 0 0"/>
     </joint>

     <link name="left_camera">
       <inertial>
         <mass value="0.01"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
       </inertial>
     </link>

     <!-- Right eye camera -->
     <joint name="right_camera_joint" type="fixed">
       <parent link="head"/>
       <child link="right_camera"/>
       <origin xyz="0.05 -0.05 0" rpy="0 0 0"/>
     </joint>

     <link name="right_camera">
       <inertial>
         <mass value="0.01"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
       </inertial>
     </link>

     <!-- IMU Sensor -->
     <joint name="imu_joint" type="fixed">
       <parent link="base_link"/>
       <child link="imu_link"/>
       <origin xyz="0 0 0.2" rpy="0 0 0"/>
     </joint>

     <link name="imu_link">
       <inertial>
         <mass value="0.01"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
       </inertial>
     </link>

     <!-- Gazebo plugins for sensors -->
     <!-- Left camera -->
     <gazebo reference="left_camera">
       <sensor type="camera" name="left_camera_sensor">
         <update_rate>30</update_rate>
         <camera name="left_camera">
           <horizontal_fov>1.3962634</horizontal_fov> <!-- 80 degrees -->
           <image>
             <width>640</width>
             <height>480</height>
             <format>R8G8B8</format>
           </image>
           <clip>
             <near>0.1</near>
             <far>100</far>
           </clip>
           <noise>
             <type>gaussian</type>
             <mean>0.0</mean>
             <stddev>0.007</stddev>
           </noise>
         </camera>
         <plugin name="left_camera_controller" filename="libgazebo_ros_camera.so">
           <cameraName>stereo/left</cameraName>
           <imageTopicName>image_raw</imageTopicName>
           <cameraInfoTopicName>camera_info</cameraInfoTopicName>
           <frameName>left_camera</frameName>
           <hackBaseline>0.07</hackBaseline>
           <distortion_k1>0.0</distortion_k1>
           <distortion_k2>0.0</distortion_k2>
           <distortion_k3>0.0</distortion_k3>
           <distortion_t1>0.0</distortion_t1>
           <distortion_t2>0.0</distortion_t2>
         </plugin>
       </sensor>
     </gazebo>

     <!-- Right camera -->
     <gazebo reference="right_camera">
       <sensor type="camera" name="right_camera_sensor">
         <update_rate>30</update_rate>
         <camera name="right_camera">
           <horizontal_fov>1.3962634</horizontal_fov> <!-- 80 degrees -->
           <image>
             <width>640</width>
             <height>480</height>
             <format>R8G8B8</format>
           </image>
           <clip>
             <near>0.1</near>
             <far>100</far>
           </clip>
           <noise>
             <type>gaussian</type>
             <mean>0.0</mean>
             <stddev>0.007</stddev>
           </noise>
         </camera>
         <plugin name="right_camera_controller" filename="libgazebo_ros_camera.so">
           <cameraName>stereo/right</cameraName>
           <imageTopicName>image_raw</imageTopicName>
           <cameraInfoTopicName>camera_info</cameraInfoTopicName>
           <frameName>right_camera</frameName>
           <hackBaseline>0.07</hackBaseline>
           <distortion_k1>0.0</distortion_k1>
           <distortion_k2>0.0</distortion_k2>
           <distortion_k3>0.0</distortion_k3>
           <distortion_t1>0.0</distortion_t1>
           <distortion_t2>0.0</distortion_t2>
         </plugin>
       </sensor>
     </gazebo>

     <!-- IMU Sensor -->
     <gazebo reference="imu_link">
       <sensor type="imu" name="imu_sensor">
         <always_on>true</always_on>
         <update_rate>100</update_rate>
         <imu>
           <angular_velocity>
             <x>
               <noise type="gaussian">
                 <mean>0.0</mean>
                 <stddev>0.0017</stddev>
               </noise>
             </x>
             <y>
               <noise type="gaussian">
                 <mean>0.0</mean>
                 <stddev>0.0017</stddev>
               </noise>
             </y>
             <z>
               <noise type="gaussian">
                 <mean>0.0</mean>
                 <stddev>0.0017</stddev>
               </noise>
             </z>
           </angular_velocity>
           <linear_acceleration>
             <x>
               <noise type="gaussian">
                 <mean>0.0</mean>
                 <stddev>1.7e-2</stddev>
               </noise>
             </x>
             <y>
               <noise type="gaussian">
                 <mean>0.0</mean>
                 <stddev>1.7e-2</stddev>
               </noise>
             </y>
             <z>
               <noise type="gaussian">
                 <mean>0.0</mean>
                 <stddev>1.7e-2</stddev>
               </noise>
             </z>
           </linear_acceleration>
         </imu>
         <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
           <topicName>imu/data</topicName>
           <serviceName>imu/service</serviceName>
           <gaussianNoise>0.0017</gaussianNoise>
           <frameName>imu_link</frameName>
         </plugin>
       </sensor>
     </gazebo>

     <!-- LiDAR -->
     <joint name="lidar_joint" type="fixed">
       <parent link="head"/>
       <child link="lidar_link"/>
       <origin xyz="0 0 0.15" rpy="0 0 0"/>
     </joint>

     <link name="lidar_link">
       <visual>
         <geometry>
           <cylinder radius="0.05" length="0.05"/>
         </geometry>
         <material name="red">
           <color rgba="1 0 0 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.05" length="0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.1"/>
         <origin xyz="0 0 0"/>
         <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
       </inertial>
     </link>

     <!-- LiDAR Sensor -->
     <gazebo reference="lidar_link">
       <sensor type="ray" name="lidar_sensor">
         <always_on>true</always_on>
         <update_rate>10</update_rate>
         <ray>
           <scan>
             <horizontal>
               <samples>720</samples>
               <resolution>1</resolution>
               <min_angle>-1.570796</min_angle>
               <max_angle>1.570796</max_angle>
             </horizontal>
           </scan>
           <range>
             <min>0.1</min>
             <max>10.0</max>
             <resolution>0.01</resolution>
           </range>
         </ray>
         <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
           <topicName>scan</topicName>
           <frameName>lidar_link</frameName>
         </plugin>
       </sensor>
     </gazebo>
   </robot>
   ```

2. **Create a Sensor Visualization Script**:
   ```python
   #!/usr/bin/env python3
   """
   Script to visualize sensor data from the humanoid robot
   """
   import rospy
   import cv2
   import numpy as np
   from sensor_msgs.msg import Image, LaserScan, Imu
   from cv_bridge import CvBridge
   import matplotlib.pyplot as plt
   from matplotlib.animation import FuncAnimation

   class SensorVisualizer:
       def __init__(self):
           rospy.init_node('sensor_visualizer', anonymous=True)
           self.bridge = CvBridge()

           # Subscribe to sensor topics
           self.image_sub = rospy.Subscriber('/stereo/left/image_raw', Image, self.image_callback)
           self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
           self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

           # Store sensor data
           self.latest_image = None
           self.scan_ranges = None
           self.imu_orientation = None

           # Set up visualization
           self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(1, 3, figsize=(15, 5))
           plt.ion()  # Interactive mode

       def image_callback(self, data):
           try:
               cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
               self.latest_image = cv_image
           except Exception as e:
               rospy.logerr(f"Error converting image: {e}")

       def scan_callback(self, data):
           self.scan_ranges = np.array(data.ranges)
           # Replace invalid ranges with max range
           self.scan_ranges[np.isnan(self.scan_ranges)] = data.range_max
           self.scan_ranges[self.scan_ranges > data.range_max] = data.range_max

       def imu_callback(self, data):
           # Extract orientation from quaternion
           w, x, y, z = data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z

           # Convert to roll, pitch, yaw (simplified)
           sinr_cosp = 2 * (w * x + y * z)
           cosr_cosp = 1 - 2 * (x * x + y * y)
           roll = np.arctan2(sinr_cosp, cosr_cosp)

           sinp = 2 * (w * y - z * x)
           if abs(sinp) >= 1:
               pitch = np.copysign(np.pi / 2, sinp)
           else:
               pitch = np.arcsin(sinp)

           siny_cosp = 2 * (w * z + x * y)
           cosy_cosp = 1 - 2 * (y * y + z * z)
           yaw = np.arctan2(siny_cosp, cosy_cosp)

           self.imu_orientation = (roll, pitch, yaw)

       def update_visualization(self, frame):
           # Clear axes
           self.ax1.clear()
           self.ax2.clear()
           self.ax3.clear()

           # Display image
           if self.latest_image is not None:
               self.ax1.imshow(cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2RGB))
               self.ax1.set_title('Camera Image')
               self.ax1.axis('off')
           else:
               self.ax1.text(0.5, 0.5, 'No Image', horizontalalignment='center', verticalalignment='center')
               self.ax1.set_title('Camera Image')
               self.ax1.axis('off')

           # Display LiDAR scan
           if self.scan_ranges is not None:
               angles = np.linspace(-np.pi/2, np.pi/2, len(self.scan_ranges))
               self.ax2.plot(angles, self.scan_ranges)
               self.ax2.set_title('LiDAR Scan')
               self.ax2.set_xlabel('Angle (rad)')
               self.ax2.set_ylabel('Distance (m)')
           else:
               self.ax2.text(0.5, 0.5, 'No Scan Data', horizontalalignment='center', verticalalignment='center')
               self.ax2.set_title('LiDAR Scan')

           # Display IMU data
           if self.imu_orientation is not None:
               roll, pitch, yaw = self.imu_orientation
               imu_data = [np.degrees(roll), np.degrees(pitch), np.degrees(yaw)]
               labels = ['Roll', 'Pitch', 'Yaw']
               self.ax3.bar(labels, imu_data)
               self.ax3.set_title('IMU Orientation (degrees)')
               self.ax3.set_ylabel('Angle (deg)')
           else:
               self.ax3.text(0.5, 0.5, 'No IMU Data', horizontalalignment='center', verticalalignment='center')
               self.ax3.set_title('IMU Orientation')

           plt.tight_layout()

       def run(self):
           # Set up animation
           ani = FuncAnimation(self.fig, self.update_visualization, interval=100)
           plt.show()

           # Keep the node alive
           rospy.spin()

   if __name__ == '__main__':
       visualizer = SensorVisualizer()
       visualizer.run()
   ```

3. **Create a Launch File for the Sensor-Equipped Robot**:
   ```xml
   <!-- launch/sensor_demo.launch -->
   <launch>
     <!-- Load robot description -->
     <param name="robot_description" command="$(find xacro)/xacro '$(find your_robot_description)/urdf/humanoid_with_sensors.urdf.xacro'" />

     <!-- Launch Gazebo -->
     <include file="$(find gazebo_ros)/launch/empty_world.launch">
       <arg name="paused" value="false"/>
       <arg name="use_sim_time" value="true"/>
       <arg name="gui" value="true"/>
       <arg name="headless" value="false"/>
       <arg name="debug" value="false"/>
     </include>

     <!-- Spawn the robot -->
     <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
           args="-param robot_description -urdf -model humanoid_with_sensors -x 0 -y 0 -z 1.0" />

     <!-- Robot state publisher -->
     <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
   </launch>
   ```

4. **Test the Sensor Configuration**:
   ```bash
   # Launch the sensor-equipped robot
   roslaunch your_robot_gazebo sensor_demo.launch

   # In another terminal, run the sensor visualizer
   rosrun your_robot_sensors sensor_visualizer.py
   ```

**Expected Outcome**: A humanoid robot with multiple sensor types (stereo camera, LiDAR, IMU) that provides realistic sensor data.

**Learning Points**:
- Understanding how to configure multiple sensors on a robot
- Learning to process and visualize different sensor data types
- Recognizing the importance of proper sensor placement on a humanoid

### Assignment 2: Implementing Sensor Fusion for Enhanced Perception

**Objective**: Combine data from multiple sensors to create a more comprehensive understanding of the environment.

**Detailed Steps**:
1. **Create a Sensor Fusion Node**:
   ```python
   #!/usr/bin/env python3
   """
   Sensor fusion node that combines data from multiple sensors
   """
   import rospy
   import numpy as np
   import tf
   from sensor_msgs.msg import Image, LaserScan, Imu
   from geometry_msgs.msg import PointStamped
   from visualization_msgs.msg import Marker
   from cv_bridge import CvBridge
   from tf.transformations import euler_from_quaternion

   class SensorFusionNode:
       def __init__(self):
           rospy.init_node('sensor_fusion_node', anonymous=True)
           self.bridge = CvBridge()

           # TF listener
           self.tf_listener = tf.TransformListener()

           # Subscribe to sensor topics
           self.image_sub = rospy.Subscriber('/stereo/left/image_raw', Image, self.image_callback)
           self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
           self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

           # Publisher for fused data
           self.obstacle_pub = rospy.Publisher('/fused_obstacles', Marker, queue_size=10)
           self.position_pub = rospy.Publisher('/fused_position', PointStamped, queue_size=10)

           # Store sensor data
           self.latest_image = None
           self.scan_ranges = None
           self.imu_data = None

           # Set processing rate
           self.rate = rospy.Rate(10)  # 10 Hz

       def image_callback(self, data):
           try:
               cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
               self.latest_image = cv_image
           except Exception as e:
               rospy.logerr(f"Error converting image: {e}")

       def scan_callback(self, data):
           self.scan_ranges = np.array(data.ranges)
           # Replace invalid ranges with max range
           self.scan_ranges[np.isnan(self.scan_ranges)] = data.range_max
           self.scan_ranges[self.scan_ranges > data.range_max] = data.range_max

       def imu_callback(self, data):
           self.imu_data = data

       def process_sensor_data(self):
           # Example: Combine LiDAR and IMU data to estimate robot pose
           if self.scan_ranges is not None and self.imu_data is not None:
               # Extract orientation from IMU
               orientation_q = [
                   self.imu_data.orientation.x,
                   self.imu_data.orientation.y,
                   self.imu_data.orientation.z,
                   self.imu_data.orientation.w
               ]
               roll, pitch, yaw = euler_from_quaternion(orientation_q)

               # Process LiDAR data to detect obstacles
               obstacle_distances = self.scan_ranges[self.scan_ranges < 2.0]  # Objects within 2 meters

               # Publish visualization marker for obstacles
               if len(obstacle_distances) > 0:
                   marker = Marker()
                   marker.header.frame_id = "base_link"
                   marker.header.stamp = rospy.Time.now()
                   marker.ns = "obstacles"
                   marker.id = 0
                   marker.type = Marker.SPHERE_LIST
                   marker.action = Marker.ADD
                   marker.pose.orientation.w = 1.0
                   marker.scale.x = 0.1
                   marker.scale.y = 0.1
                   marker.scale.z = 0.1
                   marker.color.a = 1.0
                   marker.color.r = 1.0

                   # Calculate positions of obstacles in robot frame
                   angles = np.linspace(-np.pi/2, np.pi/2, len(self.scan_ranges))
                   for i, dist in enumerate(self.scan_ranges):
                       if dist < 2.0:  # Within 2 meters
                           angle = angles[i]
                           point = PointStamped()
                           point.header.frame_id = "lidar_link"
                           point.point.x = dist * np.cos(angle)
                           point.point.y = dist * np.sin(angle)
                           point.point.z = 0.0

                           # Transform to base_link frame
                           try:
                               point_base = self.tf_listener.transformPoint("base_link", point)
                               marker.points.append(point_base.point)
                           except:
                               # If transform fails, add in lidar frame
                               marker.points.append(point.point)

                   self.obstacle_pub.publish(marker)

       def run(self):
           while not rospy.is_shutdown():
               self.process_sensor_data()
               self.rate.sleep()

   if __name__ == '__main__':
       fusion_node = SensorFusionNode()
       fusion_node.run()
   ```

2. **Create a Launch File for Sensor Fusion**:
   ```xml
   <!-- launch/sensor_fusion_demo.launch -->
   <launch>
     <!-- Load robot description -->
     <param name="robot_description" command="$(find xacro)/xacro '$(find your_robot_description)/urdf/humanoid_with_sensors.urdf.xacro'" />

     <!-- Launch Gazebo -->
     <include file="$(find gazebo_ros)/launch/empty_world.launch">
       <arg name="paused" value="false"/>
       <arg name="use_sim_time" value="true"/>
       <arg name="gui" value="true"/>
       <arg name="headless" value="false"/>
       <arg name="debug" value="false"/>
     </include>

     <!-- Spawn the robot -->
     <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
           args="-param robot_description -urdf -model humanoid_with_sensors -x 0 -y 0 -z 1.0" />

     <!-- Robot state publisher -->
     <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

     <!-- Sensor fusion node -->
     <node name="sensor_fusion_node" pkg="your_robot_sensors" type="sensor_fusion_node.py" output="screen" />
   </launch>
   ```

3. **Test Sensor Fusion**:
   ```bash
   # Launch the sensor fusion demo
   roslaunch your_robot_gazebo sensor_fusion_demo.launch

   # Visualize in RViz
   rosrun rviz rviz
   # Add marker display and set the topic to /fused_obstacles
   ```

**Expected Outcome**: A system that combines data from multiple sensors to create a unified perception of the environment.

**Learning Points**:
- Understanding how to fuse data from multiple sensors
- Learning to process and combine different types of sensor data
- Recognizing the benefits of sensor fusion for robust perception

### Assignment 3: Implementing Sensor Noise Models for Realism

**Objective**: Add realistic noise models to sensor data to better simulate real-world conditions.

**Detailed Steps**:
1. **Create a Noise Model Implementation**:
   ```python
   #!/usr/bin/env python3
   """
   Node to add realistic noise to sensor data
   """
   import rospy
   import numpy as np
   from sensor_msgs.msg import LaserScan, Imu
   from std_msgs.msg import Header

   class SensorNoiseModel:
       def __init__(self):
           rospy.init_node('sensor_noise_model', anonymous=True)

           # Subscribe to raw sensor data
           self.scan_sub = rospy.Subscriber('/raw_scan', LaserScan, self.scan_noise_callback)
           self.imu_sub = rospy.Subscriber('/raw_imu', Imu, self.imu_noise_callback)

           # Publishers for noisy sensor data
           self.noisy_scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
           self.noisy_imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)

           # Noise parameters for LiDAR
           self.lidar_range_noise_std = 0.02  # 2cm standard deviation
           self.lidar_systematic_error = 0.01  # 1cm systematic error

           # Noise parameters for IMU
           self.imu_ang_vel_noise_std = 0.01  # 0.01 rad/s standard deviation
           self.imu_lin_acc_noise_std = 0.1    # 0.1 m/s² standard deviation

       def add_lidar_noise(self, scan_msg):
           """Add realistic noise to LiDAR scan data"""
           noisy_ranges = []

           for range_val in scan_msg.ranges:
               if np.isnan(range_val) or range_val > scan_msg.range_max or range_val < scan_msg.range_min:
                   # Keep invalid values as they are
                   noisy_ranges.append(range_val)
               else:
                   # Add Gaussian noise
                   noise = np.random.normal(0, self.lidar_range_noise_std)
                   systematic_error = self.lidar_systematic_error
                   noisy_range = range_val + noise + systematic_error

                   # Constrain to valid range
                   noisy_range = max(scan_msg.range_min, min(scan_msg.range_max, noisy_range))
                   noisy_ranges.append(noisy_range)

           # Create new message with noisy data
           noisy_msg = LaserScan()
           noisy_msg.header = scan_msg.header
           noisy_msg.angle_min = scan_msg.angle_min
           noisy_msg.angle_max = scan_msg.angle_max
           noisy_msg.angle_increment = scan_msg.angle_increment
           noisy_msg.time_increment = scan_msg.time_increment
           noisy_msg.scan_time = scan_msg.scan_time
           noisy_msg.range_min = scan_msg.range_min
           noisy_msg.range_max = scan_msg.range_max
           noisy_msg.ranges = noisy_ranges
           noisy_msg.intensities = scan_msg.intensities  # Copy intensities if any

           return noisy_msg

       def add_imu_noise(self, imu_msg):
           """Add realistic noise to IMU data"""
           # Create new message
           noisy_msg = Imu()
           noisy_msg.header = imu_msg.header

           # Copy orientation (assuming it's relatively accurate)
           noisy_msg.orientation = imu_msg.orientation
           noisy_msg.orientation_covariance = imu_msg.orientation_covariance

           # Add noise to angular velocity
           noisy_msg.angular_velocity.x = imu_msg.angular_velocity.x + \
                                          np.random.normal(0, self.imu_ang_vel_noise_std)
           noisy_msg.angular_velocity.y = imu_msg.angular_velocity.y + \
                                          np.random.normal(0, self.imu_ang_vel_noise_std)
           noisy_msg.angular_velocity.z = imu_msg.angular_velocity.z + \
                                          np.random.normal(0, self.imu_ang_vel_noise_std)

           # Add noise to linear acceleration
           noisy_msg.linear_acceleration.x = imu_msg.linear_acceleration.x + \
                                            np.random.normal(0, self.imu_lin_acc_noise_std)
           noisy_msg.linear_acceleration.y = imu_msg.linear_acceleration.y + \
                                            np.random.normal(0, self.imu_lin_acc_noise_std)
           noisy_msg.linear_acceleration.z = imu_msg.linear_acceleration.z + \
                                            np.random.normal(0, self.imu_lin_acc_noise_std)

           # Set covariance values (indicating uncertainty)
           noisy_msg.angular_velocity_covariance = [self.imu_ang_vel_noise_std**2, 0, 0,
                                                   0, self.imu_ang_vel_noise_std**2, 0,
                                                   0, 0, self.imu_ang_vel_noise_std**2]
           noisy_msg.linear_acceleration_covariance = [self.imu_lin_acc_noise_std**2, 0, 0,
                                                      0, self.imu_lin_acc_noise_std**2, 0,
                                                      0, 0, self.imu_lin_acc_noise_std**2]

           return noisy_msg

       def scan_noise_callback(self, scan_msg):
           """Process LiDAR scan with noise"""
           noisy_scan = self.add_lidar_noise(scan_msg)
           self.noisy_scan_pub.publish(noisy_scan)

       def imu_noise_callback(self, imu_msg):
           """Process IMU data with noise"""
           noisy_imu = self.add_imu_noise(imu_msg)
           self.noisy_imu_pub.publish(noisy_imu)

       def run(self):
           rospy.loginfo("Sensor noise model started")
           rospy.spin()

   if __name__ == '__main__':
       noise_model = SensorNoiseModel()
       noise_model.run()
   ```

2. **Create a Sensor Validation Node**:
   ```python
   #!/usr/bin/env python3
   """
   Node to validate sensor data quality
   """
   import rospy
   import numpy as np
   from sensor_msgs.msg import LaserScan, Imu
   from std_msgs.msg import Float32

   class SensorValidator:
       def __init__(self):
           rospy.init_node('sensor_validator', anonymous=True)

           # Subscribe to sensor data
           self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_validation_callback)
           self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_validation_callback)

           # Publishers for validation metrics
           self.scan_quality_pub = rospy.Publisher('/scan_quality', Float32, queue_size=10)
           self.imu_stability_pub = rospy.Publisher('/imu_stability', Float32, queue_size=10)

           # Store historical data for validation
           self.scan_history = []
           self.imu_history = []
           self.max_history = 100

       def calculate_scan_quality(self, scan_msg):
           """Calculate quality metric for LiDAR scan"""
           # Filter out invalid ranges
           valid_ranges = [r for r in scan_msg.ranges if not (np.isnan(r) or r > scan_msg.range_max)]

           if len(valid_ranges) == 0:
               return 0.0

           # Calculate some metrics
           avg_range = np.mean(valid_ranges)
           range_variance = np.var(valid_ranges)
           valid_ratio = len(valid_ranges) / len(scan_msg.ranges)

           # Quality score based on multiple factors
           # Higher score indicates better quality
           quality_score = (valid_ratio * 0.5) + (1.0 / (1.0 + range_variance) * 0.3) + \
                          (min(1.0, 10.0 / avg_range) * 0.2)

           return min(1.0, quality_score)

       def calculate_imu_stability(self, imu_msg):
           """Calculate stability metric for IMU data"""
           # Convert to numpy arrays
           current_ang_vel = np.array([imu_msg.angular_velocity.x,
                                      imu_msg.angular_velocity.y,
                                      imu_msg.angular_velocity.z])
           current_lin_acc = np.array([imu_msg.linear_acceleration.x,
                                      imu_msg.linear_acceleration.y,
                                      imu_msg.linear_acceleration.z])

           # Calculate magnitude
           ang_vel_magnitude = np.linalg.norm(current_ang_vel)
           lin_acc_magnitude = np.linalg.norm(current_lin_acc)

           # Stability is inversely related to motion (lower values indicate more stable)
           stability = 1.0 / (1.0 + ang_vel_magnitude + lin_acc_magnitude)

           return min(1.0, stability)

       def scan_validation_callback(self, scan_msg):
           """Validate LiDAR scan quality"""
           quality = self.calculate_scan_quality(scan_msg)

           # Publish quality metric
           quality_msg = Float32()
           quality_msg.data = quality
           self.scan_quality_pub.publish(quality_msg)

           # Store in history
           self.scan_history.append(quality)
           if len(self.scan_history) > self.max_history:
               self.scan_history.pop(0)

           # Log if quality is low
           if quality < 0.3:
               rospy.logwarn(f"Low LiDAR quality detected: {quality:.2f}")

       def imu_validation_callback(self, imu_msg):
           """Validate IMU stability"""
           stability = self.calculate_imu_stability(imu_msg)

           # Publish stability metric
           stability_msg = Float32()
           stability_msg.data = stability
           self.imu_stability_pub.publish(stability_msg)

           # Store in history
           self.imu_history.append(stability)
           if len(self.imu_history) > self.max_history:
               self.imu_history.pop(0)

           # Log if unstable
           if stability < 0.2:
               rospy.logwarn(f"IMU instability detected: {stability:.2f}")

       def run(self):
           rospy.loginfo("Sensor validator started")
           rospy.spin()

   if __name__ == '__main__':
       validator = SensorValidator()
       validator.run()
   ```

3. **Test Noise Models and Validation**:
   ```bash
   # Create a launch file combining everything
   # launch/noise_validation_demo.launch
   <launch>
     <!-- Load robot description -->
     <param name="robot_description" command="$(find xacro)/xacro '$(find your_robot_description)/urdf/humanoid_with_sensors.urdf.xacro'" />

     <!-- Launch Gazebo -->
     <include file="$(find gazebo_ros)/launch/empty_world.launch">
       <arg name="paused" value="false"/>
       <arg name="use_sim_time" value="true"/>
       <arg name="gui" value="true"/>
       <arg name="headless" value="false"/>
       <arg name="debug" value="false"/>
     </include>

     <!-- Spawn the robot -->
     <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
           args="-param robot_description -urdf -model humanoid_with_sensors -x 0 -y 0 -z 1.0" />

     <!-- Robot state publisher -->
     <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

     <!-- Sensor noise model -->
     <node name="sensor_noise_model" pkg="your_robot_sensors" type="sensor_noise_model.py" output="screen" />

     <!-- Sensor validator -->
     <node name="sensor_validator" pkg="your_robot_sensors" type="sensor_validator.py" output="screen" />
   </launch>

   # Run the demo
   roslaunch your_robot_gazebo noise_validation_demo.launch

   # Monitor the sensor quality topics
   rostopic echo /scan_quality
   rostopic echo /imu_stability
   ```

**Expected Outcome**: A comprehensive sensor system with realistic noise models and validation capabilities that provide quality metrics.

**Learning Points**:
- Understanding how to implement realistic noise models for sensors
- Learning to validate sensor data quality
- Recognizing the importance of noise modeling in simulation

## Troubleshooting Sensor Issues

### LiDAR Problems
- **Problem**: Missing or inconsistent scan data
- **Solution**: Check sensor configuration, verify collision/mesh geometry, adjust update rate

### Camera Issues
- **Problem**: Black or distorted images
- **Solution**: Verify camera parameters, check lighting in scene, validate image format

### IMU Drift
- **Problem**: IMU readings drift over time
- **Solution**: Implement sensor fusion with other sensors, improve noise models