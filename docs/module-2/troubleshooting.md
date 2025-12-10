---
sidebar_position: 6
title: Troubleshooting the Digital Twin Simulation
---

# Troubleshooting Digital Twin Simulation

This section helps identify and resolve common issues encountered when running the Digital Twin simulation with Gazebo and Unity.

## Common Issues and Solutions

### Gazebo Physics Issues

#### 1. Object Falling Through Ground
**Symptoms**: Robot or objects fall through the ground plane
**Solution**:
- Check collision geometry definition in URDF/SDF
- Increase contact surface layer in physics parameters
- Verify mass and inertia properties of model

#### 2. Unstable Physics Simulation
**Symptoms**: Objects vibrating, exploding, or behaving erratically
**Solution**:
- Reduce `max_step_size` in physics config (e.g., to 0.001)
- Adjust solver parameters (iterations,SOR factor)
- Verify mass and inertia values are physically plausible

#### 3. Gravity Not Working Properly
**Symptoms**: Objects not falling at expected rate
**Solution**:
- Check that gravity is set to 0 0 -9.81 in world file
- Verify physics engine configuration
- Test with simple test object

### Unity Visualization Issues

#### 1. Low Performance (FPS Below 30)
**Symptoms**: Slow rendering or frame rate drops
**Solutions**:
- Check graphics driver updates
- Review Unity Quality Settings (Edit > Project Settings > Quality)
- Simplify complex scenes temporarily
- Disable unnecessary render features

#### 2. Synchronization Lag
**Symptoms**: Unity scene not reflecting Gazebo state in real-time
**Solutions**:
- Check ROS communication between systems
- Verify transform synchronization nodes
- Monitor network latency
- Increase update rate on synchronization

#### 3. Visual Discrepancies
**Symptoms**: Differences in appearance between Gazebo and Unity
**Solutions**:
- Ensure identical lighting setups
- Verify coordinate system alignment
- Check for scaling differences
- Validate texture/material mappings

### Sensor Simulation Issues

#### 1. LiDAR Not Publishing Data
**Symptoms**: No data on `/lidar/scan` topic
**Solutions**:
- Verify sensor plugin in URDF/SDF file
- Check that Gazebo plugins are loaded
- Confirm ROS bridge is connected
- Check topic existence: `rostopic list | grep lidar`

#### 2. Depth Camera Output Invalid
**Symptoms**: All depth values infinity, zero, or NaN
**Solutions**:
- Check camera plugin configuration
- Verify image format compatibility
- Test with simple scene (no occlusions)
- Confirm coordinate frame conventions

#### 3. IMU Showing Constant Values
**Symptoms**: No change in IMU readings despite motion
**Solutions**:
- Verify IMU sensor is properly attached to model
- Check that IMU plugin is correctly configured
- Confirm model is actually moving in simulation

### ROS Communication Issues

#### 1. ROS Bridge Connection Problems
**Symptoms**: Unity not receiving Gazebo data or vice versa
**Solutions**:
- Check ROS_MASTER_URI and ROS_IP environment variables
- Verify ROS bridge server is running
- Test basic communication with `rostopic echo`
- Check firewall settings

#### 2. Topic Names Mismatch
**Symptoms**: Data not flowing to expected topics
**Solutions**:
- Verify topic names in sensor plugins match subscriber expectations
- Use `rostopic list` to see available topics
- Check namespace settings in plugin configurations

## Performance Optimization

### Gazebo Optimization
- Reduce physics update rate if not critical (1000Hz → 500Hz)
- Simplify collision meshes
- Limit number of active sensors
- Use `static` property for non-moving objects

### Unity Optimization
- Implement occlusion culling
- Use object pooling for frequently instantiated items
- Optimize texture sizes
- Reduce real-time lighting calculations

### Network Optimization
- Use localhost communications when possible
- Minimize message size through filtering
- Check network buffer settings
- Consider message rate limiting for non-critical data

## Diagnostic Procedures

### Physics Validation
```bash
# Verify gravity with a simple test
ros2 launch gazebo_ros empty_world.launch.py
ros2 run gazebo_ros spawn_entity.py -entity test_ball -x 0 -y 0 -z 5
# Check if ball falls at 9.81 m/s^2
```

### Sensor Verification
```bash
# Verify sensor topics are publishing
rostopic echo /lidar/scan --field=ranges[0] -n1
rostopic echo /depth_cam/image_raw --field=header.stamp -n1
rostopic echo /imu/data --field=linear_acceleration.z -n1
```

### Synchronization Check
```bash
# Monitor synchronization accuracy
rostopic hz /gazebo/robot_pose
rostopic hz /unity/robot_pose
# Look for timing differences
```

## Debugging Tips

### Enable Verbose Logging
In Gazebo launch files, add verbose flag:
```bash
# Launch with verbose output
gazebo --verbose your_world.world
```

### Use RViz for Visualization
```bash
# Visualize sensor data
rviz2 -d /path/to/sensor_visualization.rviz
```

### Monitor Resource Usage
```bash
# Monitor system resources during simulation
htop
nvidia-smi  # For GPU monitoring
```

## Advanced Troubleshooting

### When Nothing Works
1. Restart all systems: Gazebo, Unity, ROS nodes
2. Verify environment variables are set correctly
3. Check hardware requirements are met
4. Run minimal test scenarios to isolate issues

### Contact and Support Resources
- Consult the ROS and Gazebo documentation
- Check Unity forums for rendering issues
- Verify hardware compatibility with minimum requirements
- Consider posting in robotics simulation communities

## Practical Assignment - Troubleshooting Common Simulation Issues

### Assignment 1: Performance Diagnosis and Optimization

**Objective**: Identify and resolve performance bottlenecks in the digital twin simulation.

**Detailed Steps**:
1. **Create a Performance Monitoring Tool**:
   ```python
   #!/usr/bin/env python3
   """
   Performance monitoring tool for digital twin simulation
   """
   import rospy
   import psutil
   import time
   from std_msgs.msg import Float32
   from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
   import subprocess

   class PerformanceMonitor:
       def __init__(self):
           rospy.init_node('performance_monitor', anonymous=True)

           # Publishers for performance metrics
           self.cpu_pub = rospy.Publisher('/diagnostics/cpu_usage', Float32, queue_size=10)
           self.mem_pub = rospy.Publisher('/diagnostics/memory_usage', Float32, queue_size=10)
           self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

           # Parameters
           self.monitoring_rate = rospy.get_param('~monitoring_rate', 1.0)  # Hz
           self.gpu_monitoring_enabled = rospy.get_param('~enable_gpu_monitoring', False)

           # Rate controller
           self.rate = rospy.Rate(self.monitoring_rate)

       def get_gpu_usage(self):
           """Get GPU utilization if nvidia-smi is available"""
           try:
               result = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu,memory.used,memory.total',
                                      '--format=csv,noheader,nounits'],
                                    capture_output=True, text=True)
               if result.returncode == 0:
                   gpu_data = result.stdout.strip().split(', ')
                   gpu_util = float(gpu_data[0])
                   return gpu_util
           except FileNotFoundError:
               rospy.logwarn("nvidia-smi not found, skipping GPU monitoring")
           except Exception as e:
               rospy.logwarn(f"Error getting GPU stats: {e}")

           return -1  # Indicate no GPU info available

       def publish_diagnostics(self):
           """Publish diagnostic information"""
           diag_array = DiagnosticArray()
           diag_array.header.stamp = rospy.Time.now()

           # CPU diagnostic
           cpu_diag = DiagnosticStatus()
           cpu_diag.name = "CPU Usage"
           cpu_percent = psutil.cpu_percent(interval=1)
           cpu_diag.values.append(KeyValue(key="Percentage", value=f"{cpu_percent}%"))
           if cpu_percent > 80:
               cpu_diag.level = DiagnosticStatus.WARN
               cpu_diag.message = f"High CPU usage: {cpu_percent}%"
           elif cpu_percent > 90:
               cpu_diag.level = DiagnosticStatus.ERROR
               cpu_diag.message = f"Critical CPU usage: {cpu_percent}%"
           else:
               cpu_diag.level = DiagnosticStatus.OK
               cpu_diag.message = f"Normal CPU usage: {cpu_percent}%"
           diag_array.status.append(cpu_diag)

           # Memory diagnostic
           mem_diag = DiagnosticStatus()
           mem_diag.name = "Memory Usage"
           memory = psutil.virtual_memory()
           mem_percent = memory.percent
           mem_diag.values.append(KeyValue(key="Percentage", value=f"{mem_percent}%"))
           mem_diag.values.append(KeyValue(key="Available MB", value=str(memory.available // (1024*1024))))
           if mem_percent > 85:
               mem_diag.level = DiagnosticStatus.WARN
               mem_diag.message = f"High memory usage: {mem_percent}%"
           elif mem_percent > 95:
               mem_diag.level = DiagnosticStatus.ERROR
               mem_diag.message = f"Critical memory usage: {mem_percent}%"
           else:
               mem_diag.level = DiagnosticStatus.OK
               mem_diag.message = f"Normal memory usage: {mem_percent}%"
           diag_array.status.append(mem_diag)

           # GPU diagnostic (if available)
           if self.gpu_monitoring_enabled:
               gpu_usage = self.get_gpu_usage()
               if gpu_usage >= 0:
                   gpu_diag = DiagnosticStatus()
                   gpu_diag.name = "GPU Usage"
                   gpu_diag.values.append(KeyValue(key="Percentage", value=f"{gpu_usage}%"))
                   if gpu_usage > 85:
                       gpu_diag.level = DiagnosticStatus.WARN
                       gpu_diag.message = f"High GPU usage: {gpu_usage}%"
                   elif gpu_usage > 95:
                       gpu_diag.level = DiagnosticStatus.ERROR
                       gpu_diag.message = f"Critical GPU usage: {gpu_usage}%"
                   else:
                       gpu_diag.level = DiagnosticStatus.OK
                       gpu_diag.message = f"Normal GPU usage: {gpu_usage}%"
                   diag_array.status.append(gpu_diag)

           self.diag_pub.publish(diag_array)

           # Publish as separate topics too
           cpu_msg = Float32()
           cpu_msg.data = cpu_percent
           self.cpu_pub.publish(cpu_msg)

           mem_msg = Float32()
           mem_msg.data = mem_percent
           self.mem_pub.publish(mem_msg)

       def run(self):
           rospy.loginfo("Performance monitor started")
           while not rospy.is_shutdown():
               self.publish_diagnostics()
               self.rate.sleep()

   if __name__ == '__main__':
       monitor = PerformanceMonitor()
       monitor.run()
   ```

2. **Create a Gazebo Performance Profiler**:
   ```bash
   #!/bin/bash
   # gazebo_profiler.sh
   # Script to profile Gazebo simulation performance

   echo "Starting Gazebo Performance Profiling..."

   # Check if Gazebo is running
   if pgrep gzserver > /dev/null; then
       echo "Gazebo server is running. Profiling..."

       # Get Gazebo process info
       echo "Gazebo Server Process Info:"
       ps aux | grep gzserver

       # Monitor Gazebo topics
       echo -e "\nMonitoring simulation time topics..."
       timeout 10 rostopic echo /clock -n 1 2>/dev/null || echo "Clock topic not available"

       # Check common simulation topics
       echo -e "\nChecking common simulation topics..."
       rostopic list | grep -E "(scan|image|imu|joint|tf)" | head -10

       # Monitor TF tree
       echo -e "\nTF Tree Status:"
       rosrun tf tf_monitor 2>/dev/null &
       TF_PID=$!
       sleep 5
       kill $TF_PID

   else
       echo "Gazebo server is not running."
       echo "Please start Gazebo before running this profiler."
   fi
   ```

3. **Execute Performance Diagnostics**:
   ```bash
   # Run the performance monitor
   rosrun your_robot_diagnostics performance_monitor.py _monitoring_rate:=2.0 _enable_gpu_monitoring:=true

   # Run the Gazebo profiler
   chmod +x gazebo_profiler.sh
   ./gazebo_profiler.sh
   ```

**Expected Outcome**: Identification of performance bottlenecks with specific recommendations for optimization.

**Learning Points**:
- Understanding performance metrics in simulation environments
- Learning to profile and monitor system resources
- Recognizing common causes of performance issues

### Assignment 2: Synchronization Troubleshooting in Gazebo-Unity Integration

**Objective**: Diagnose and fix synchronization issues between Gazebo and Unity environments.

**Detailed Steps**:
1. **Create a Synchronization Diagnostics Tool**:
   ```python
   #!/usr/bin/env python3
   """
   Synchronization diagnostics for Gazebo-Unity integration
   """
   import rospy
   import numpy as np
   import time
   from geometry_msgs.msg import PoseStamped, TransformStamped
   from tf2_msgs.msg import TFMessage
   from std_msgs.msg import Float32, Time
   import tf2_ros
   import tf2_geometry_msgs

   class SyncDiagnostics:
       def __init__(self):
           rospy.init_node('sync_diagnostics', anonymous=True)

           # TF listener and broadcaster
           self.tf_buffer = tf2_ros.Buffer()
           self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

           # Publishers for sync diagnostics
           self.timing_diff_pub = rospy.Publisher('/diagnostics/timing_difference', Float32, queue_size=10)
           self.pos_error_pub = rospy.Publisher('/diagnostics/position_error', Float32, queue_size=10)
           self.sync_quality_pub = rospy.Publisher('/diagnostics/sync_quality', Float32, queue_size=10)

           # Subscribers
           self.gazebo_pose_sub = rospy.Subscriber('/gazebo/model_states', self.gazebo_pose_callback)
           self.unity_pose_sub = rospy.Subscriber('/unity/robot_pose', self.unity_pose_callback)

           # Store pose history for comparison
           self.gazebo_poses = {}
           self.unity_poses = {}

           self.last_gazebo_update = rospy.Time(0)
           self.last_unity_update = rospy.Time(0)

           # Timing parameters
           self.timing_threshold = rospy.Duration(rospy.get_param('~timing_threshold', 0.1))  # seconds
           self.position_threshold = rospy.get_param('~position_threshold', 0.2)  # meters

           # Rate controller
           self.rate = rospy.Rate(10)  # 10 Hz

       def gazebo_pose_callback(self, data):
           """Store Gazebo model poses"""
           current_time = rospy.Time.now()
           for i, name in enumerate(data.name):
               if i < len(data.pose):
                   self.gazebo_poses[name] = (data.pose[i], current_time)
           self.last_gazebo_update = current_time

       def unity_pose_callback(self, data):
           """Store Unity model poses"""
           current_time = rospy.Time.now()
           self.unity_poses[data.header.frame_id] = (data.pose, current_time)
           self.last_unity_update = current_time

       def calculate_sync_metrics(self):
           """Calculate synchronization metrics"""
           # Compare poses for common models
           common_models = set(self.gazebo_poses.keys()) & set(self.unity_poses.keys())

           if not common_models:
               rospy.logwarn("No common models found between Gazebo and Unity")
               return 0.0, 0.0

           position_errors = []
           timing_differences = []

           for model in common_models:
               gazebo_pose, gazebo_time = self.gazebo_poses[model]
               unity_pose, unity_time = self.unity_poses[model]

               # Calculate position difference
               pos_diff = np.sqrt(
                   (gazebo_pose.position.x - unity_pose.position.x)**2 +
                   (gazebo_pose.position.y - unity_pose.position.y)**2 +
                   (gazebo_pose.position.z - unity_pose.position.z)**2
               )
               position_errors.append(pos_diff)

               # Calculate timing difference
               time_diff = abs((gazebo_time - unity_time).to_sec())
               timing_differences.append(time_diff)

           if position_errors:
               avg_pos_error = np.mean(position_errors)
               avg_timing_error = np.mean(timing_differences)

               # Publish metrics
               pos_error_msg = Float32()
               pos_error_msg.data = avg_pos_error
               self.pos_error_pub.publish(pos_error_msg)

               timing_diff_msg = Float32()
               timing_diff_msg.data = avg_timing_error
               self.timing_diff_pub.publish(timing_diff_msg)

               # Calculate sync quality (inverse of errors)
               max_pos_error = max(position_errors) if position_errors else 0
               max_timing_error = max(timing_differences) if timing_differences else 0

               # Normalize to 0-1 scale (1 is perfect sync)
               pos_quality = max(0, 1 - (max_pos_error / self.position_threshold))
               time_quality = max(0, 1 - (max_timing_error / self.timing_threshold.to_sec()))

               sync_quality = min(pos_quality, time_quality)

               quality_msg = Float32()
               quality_msg.data = sync_quality
               self.sync_quality_pub.publish(quality_msg)

               return avg_pos_error, avg_timing_error

           return 0.0, 0.0

       def run(self):
           rospy.loginfo("Synchronization diagnostics started")
           while not rospy.is_shutdown():
               pos_error, time_error = self.calculate_sync_metrics()

               if pos_error > self.position_threshold:
                   rospy.logwarn(f"High position error detected: {pos_error:.3f}m")
               if time_error > self.timing_threshold.to_sec():
                   rospy.logwarn(f"High timing error detected: {time_error:.3f}s")

               self.rate.sleep()

   if __name__ == '__main__':
       diagnostics = SyncDiagnostics()
       diagnostics.run()
   ```

2. **Create a Synchronization Correction Node**:
   ```python
   #!/usr/bin/env python3
   """
   Synchronization correction node to fix Gazebo-Unity timing issues
   """
   import rospy
   import numpy as np
   from geometry_msgs.msg import PoseStamped, TwistStamped
   from std_msgs.msg import Float32
   import tf2_ros
   import tf2_geometry_msgs
   from scipy import interpolate

   class SyncCorrector:
       def __init__(self):
           rospy.init_node('sync_corrector', anonymous=True)

           # Parameters
           self.interpolation_window = rospy.Duration(rospy.get_param('~interpolation_window', 0.5))
           self.correction_enabled = rospy.get_param('~correction_enabled', True)

           # TF buffer
           self.tf_buffer = tf2_ros.Buffer()
           self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

           # Subscribers
           self.gazebo_sub = rospy.Subscriber('/gazebo/model_states', self.gazebo_callback)
           self.unity_sub = rospy.Subscriber('/unity/robot_pose', self.unity_callback)

           # Publishers
           self.corrected_pose_pub = rospy.Publisher('/corrected/robot_pose', PoseStamped, queue_size=10)
           self.sync_status_pub = rospy.Publisher('/diagnostics/sync_status', Float32, queue_size=10)

           # Store pose history for interpolation
           self.pose_history = []  # List of (timestamp, pose) tuples

           # Rate controller
           self.rate = rospy.Rate(100)  # 100 Hz for smooth corrections

       def gazebo_callback(self, data):
           """Process Gazebo pose data"""
           current_time = rospy.Time.now()
           for i, name in enumerate(data.name):
               if name == 'humanoid_robot':  # Assuming our robot is named 'humanoid_robot'
                   if i < len(data.pose):
                       self.add_pose_to_history(current_time, data.pose[i])
                       break

       def unity_callback(self, data):
           """Process Unity pose data"""
           # Unity data is used for comparison but doesn't directly affect corrections
           pass

       def add_pose_to_history(self, timestamp, pose):
           """Add pose to history with time window management"""
           self.pose_history.append((timestamp, pose))

           # Remove old poses outside the window
           cutoff_time = timestamp - self.interpolation_window
           self.pose_history = [(t, p) for t, p in self.pose_history if t > cutoff_time]

       def interpolate_pose(self, target_time):
           """Interpolate robot pose at target time"""
           if len(self.pose_history) < 2:
               return None

           # Extract timestamps and poses
           times = [t.to_sec() for t, p in self.pose_history]
           poses = [p for t, p in self.pose_history]

           target_time_sec = target_time.to_sec()

           # Check if target time is within range
           if target_time_sec < times[0] or target_time_sec > times[-1]:
               # Extrapolate if needed (with caution)
               rospy.logwarn("Extrapolating pose - consider adjusting interpolation window")

           # Interpolate each component separately
           target_pose = type(poses[0])()

           # Interpolate position
           pos_x = np.interp(target_time_sec, times, [p.position.x for p in poses])
           pos_y = np.interp(target_time_sec, times, [p.position.y for p in poses])
           pos_z = np.interp(target_time_sec, times, [p.position.z for p in poses])

           target_pose.position.x = pos_x
           target_pose.position.y = pos_y
           target_pose.position.z = pos_z

           # For orientation, we would typically use spherical linear interpolation (SLERP)
           # Simplified approach: just use the closest pose orientation
           closest_idx = min(range(len(times)), key=lambda i: abs(times[i] - target_time_sec))
           target_pose.orientation = poses[closest_idx].orientation

           return target_pose

       def run(self):
           rospy.loginfo("Synchronization corrector started")

           while not rospy.is_shutdown():
               if self.correction_enabled:
                   # Publish corrected pose based on current time
                   current_time = rospy.Time.now()
                   corrected_pose = self.interpolate_pose(current_time)

                   if corrected_pose is not None:
                       pose_msg = PoseStamped()
                       pose_msg.header.stamp = current_time
                       pose_msg.header.frame_id = "map"
                       pose_msg.pose = corrected_pose
                       self.corrected_pose_pub.publish(pose_msg)

                       # Publish sync quality (higher is better)
                       sync_quality = Float32()
                       sync_quality.data = 0.95 if len(self.pose_history) > 5 else 0.5
                       self.sync_status_pub.publish(sync_quality)
                   else:
                       rospy.logwarn("Not enough pose history for interpolation")
               else:
                   rospy.loginfo_throttle(10, "Synchronization correction disabled")

               self.rate.sleep()

   if __name__ == '__main__':
       corrector = SyncCorrector()
       corrector.run()
   ```

3. **Execute Synchronization Diagnostics**:
   ```bash
   # Run the synchronization diagnostics
   rosrun your_robot_diagnostics sync_diagnostics.py _timing_threshold:=0.2 _position_threshold:=0.3

   # Run the synchronization corrector
   rosrun your_robot_diagnostics sync_corrector.py _interpolation_window:=0.3 _correction_enabled:=true
   ```

**Expected Outcome**: Identification of synchronization issues and improved alignment between Gazebo and Unity environments.

**Learning Points**:
- Understanding timing and synchronization challenges in digital twins
- Learning to diagnose and correct pose discrepancies
- Recognizing the importance of real-time synchronization

### Assignment 3: Physics Simulation Troubleshooting in Humanoid Robot

**Objective**: Identify and resolve physics simulation issues specifically related to humanoid robot stability and joint behavior.

**Detailed Steps**:
1. **Create a Physics Diagnostics Tool**:
   ```python
   #!/usr/bin/env python3
   """
   Physics diagnostics tool for humanoid robot simulation
   """
   import rospy
   import numpy as np
   from sensor_msgs.msg import JointState
   from geometry_msgs.msg import WrenchStamped
   from std_msgs.msg import Float32
   from tf import TransformListener
   import tf

   class PhysicsDiagnostics:
       def __init__(self):
           rospy.init_node('physics_diagnostics', anonymous=True)

           # TF listener for pose and velocity calculations
           self.tf_listener = TransformListener()

           # Subscribers
           self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
           self.contact_force_sub = rospy.Subscriber('/contact_force', WrenchStamped, self.contact_force_callback)

           # Publishers
           self.balance_pub = rospy.Publisher('/diagnostics/balance_score', Float32, queue_size=10)
           self.joint_issue_pub = rospy.Publisher('/diagnostics/joint_issues', Float32, queue_size=10)
           self.stability_pub = rospy.Publisher('/diagnostics/stability', Float32, queue_size=10)

           # Internal state
           self.joint_states = {}
           self.contact_forces = {}
           self.base_link_velocity = None
           self.time_prev = rospy.Time.now()

           # Parameters for physics checks
           self.max_joint_velocity = rospy.get_param('~max_joint_velocity', 5.0)  # rad/s
           self.balance_threshold = rospy.get_param('~balance_threshold', 0.1)   # meters from center
           self.stability_threshold = rospy.get_param('~stability_threshold', 0.2)  # angle in radians

           # Rate controller
           self.rate = rospy.Rate(50)  # 50 Hz for physics monitoring

       def joint_state_callback(self, data):
           """Process joint state data for physics diagnostics"""
           current_time = rospy.Time.now()
           time_delta = (current_time - self.time_prev).to_sec()
           self.time_prev = current_time

           # Store current states and calculate velocities
           for i, name in enumerate(data.name):
               if i < len(data.position) and i < len(data.velocity):
                   pos = data.position[i]
                   vel = data.velocity[i]
                    # Calculate acceleration approx
                   if name in self.joint_states:
                       prev_vel = self.joint_states[name]['velocity']
                       accel = (vel - prev_vel) / time_delta if time_delta > 0 else 0.0
                   else:
                       accel = 0.0

                   self.joint_states[name] = {
                       'position': pos,
                       'velocity': vel,
                       'acceleration': accel,
                       'timestamp': current_time
                   }

       def contact_force_callback(self, data):
           """Process contact force data"""
           self.contact_forces[data.header.frame_id] = data.wrench

       def evaluate_balance(self):
           """Evaluate robot balance based on COM position"""
           try:
               # Get transform for robot base link to calculate center of mass position
               (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))

               # Simple balance calculation (distance of COM projection to support polygon)
               # For a humanoid, we'll consider the feet as the support area
               com_x, com_y = trans[0], trans[1]

               # Check feet positions (simplified - assuming fixed feet offset)
               try:
                   (left_foot_trans, _) = self.tf_listener.lookupTransform('/map', '/left_foot', rospy.Time(0))
                   (right_foot_trans, _) = self.tf_listener.lookupTransform('/map', '/right_foot', rospy.Time(0))

                   # Calculate center of support polygon
                   support_center_x = (left_foot_trans[0] + right_foot_trans[0]) / 2
                   support_center_y = (left_foot_trans[1] + right_foot_trans[1]) / 2

                   # Distance from COM to support center
                   com_distance = np.sqrt((com_x - support_center_x)**2 + (com_y - support_center_y)**2)

                   # Balance score (1 is perfect balance, 0 is unstable)
                   balance_score = max(0, 1 - (com_distance / self.balance_threshold))

                   # Publish balance score
                   balance_msg = Float32()
                   balance_msg.data = balance_score
                   self.balance_pub.publish(balance_msg)

                   return balance_score
               except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                   rospy.logwarn("Could not get foot transforms for balance calculation")
                   return 0.5  # Unknown balance state

           except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
               rospy.logwarn("Could not get base link transform for balance calculation")
               return 0.5  # Unknown balance state

       def evaluate_joint_health(self):
           """Evaluate joint health based on velocity and force"""
           joint_issue_count = 0
           total_joints = len(self.joint_states)

           for joint_name, joint_data in self.joint_states.items():
               # Check for excessive velocity
               if abs(joint_data['velocity']) > self.max_joint_velocity:
                   rospy.logwarn(f"Joint {joint_name} has excessive velocity: {joint_data['velocity']:.2f} rad/s")
                   joint_issue_count += 1

               # Check for excessive acceleration (jerk)
               if abs(joint_data['acceleration']) > 50:  # arbitrary threshold
                   rospy.logwarn(f"Joint {joint_name} has excessive acceleration: {joint_data['acceleration']:.2f} rad/s^2")
                   joint_issue_count += 1

           # Calculate joint health score
           if total_joints > 0:
               joint_health_score = 1.0 - (joint_issue_count / total_joints)
           else:
               joint_health_score = 1.0

           # Publish joint issue score
           issue_msg = Float32()
           issue_msg.data = joint_health_score
           self.joint_issue_pub.publish(issue_msg)

           return joint_health_score

       def evaluate_stability(self):
           """Evaluate overall stability based on multiple factors"""
           balance_score = self.evaluate_balance()
           joint_health_score = self.evaluate_joint_health()

           # Combine scores for overall stability
           stability_score = min(balance_score, joint_health_score)

           # Publish stability score
           stability_msg = Float32()
           stability_msg.data = stability_score
           self.stability_pub.publish(stability_msg)

           return stability_score

       def run(self):
           rospy.loginfo("Physics diagnostics started")
           while not rospy.is_shutdown():
               stability = self.evaluate_stability()

               if stability < 0.3:
                   rospy.logerr(f"Critically unstable: {stability:.2f}")
               elif stability < 0.7:
                   rospy.logwarn(f"Potentially unstable: {stability:.2f}")

               self.rate.sleep()

   if __name__ == '__main__':
       diagnostics = PhysicsDiagnostics()
       diagnostics.run()
   ```

2. **Create a Physics Correction Tool**:
   ```python
   #!/usr/bin/env python3
   """
   Physics correction tool to stabilize humanoid simulation
   """
   import rospy
   import numpy as np
   from sensor_msgs.msg import JointState
   from std_msgs.msg import Float64
   from geometry_msgs.msg import Twist

   class PhysicsCorrector:
       def __init__(self):
           rospy.init_node('physics_corrector', anonymous=True)

           # Parameters
           self.balance_correction_enabled = rospy.get_param('~balance_correction', True)
           self.joint_limit_protection = rospy.get_param('~joint_limit_protection', True)
           self.velocity_smoothing = rospy.get_param('~velocity_smoothing', True)

           # Subscribers
           self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
           self.balance_sub = rospy.Subscriber('/diagnostics/balance_score', Float32, self.balance_callback)

           # Publishers (for joint position control)
           self.joint_cmd_pubs = {}

           # Store previous states for smoothing
           self.prev_positions = {}
           self.prev_velocities = {}
           self.balance_score = 1.0

           # Correction parameters
           self.balance_threshold = 0.6  # Below this, apply corrections
           self.max_correction = 0.05  # Max position correction in radians

           # Rate controller
           self.rate = rospy.Rate(100)  # 100 Hz

       def joint_state_callback(self, data):
           """Store current joint states for correction"""
           for i, name in enumerate(data.name):
               if i < len(data.position):
                   self.prev_positions[name] = data.position[i]
                   if i < len(data.velocity):
                       self.prev_velocities[name] = data.velocity[i]

       def balance_callback(self, data):
           """Receive balance score for conditional corrections"""
           self.balance_score = data.data

       def apply_balance_correction(self):
           """Apply corrections to improve robot balance"""
           if not self.balance_correction_enabled or self.balance_score >= self.balance_threshold:
               return

           # Create correction command based on balance score
           # In practice, this would use more sophisticated control algorithms
           correction_factor = (self.balance_threshold - self.balance_score) / self.balance_threshold

           # Apply small corrective movements to joints
           for joint_name in self.prev_positions.keys():
               if "hip" in joint_name or "ankle" in joint_name:  # Correct joints that affect balance
                   cmd_pub = self.joint_cmd_pubs.get(joint_name)
                   if cmd_pub is not None:
                       # Apply small corrective movement
                       correction = correction_factor * self.max_correction * np.sign(np.random.random() - 0.5)
                       target_pos = self.prev_positions[joint_name] + correction

                       cmd_msg = Float64()
                       cmd_msg.data = target_pos
                       cmd_pub.publish(cmd_msg)

       def smooth_velocities(self):
           """Apply velocity smoothing to reduce jerky movements"""
           if not self.velocity_smoothing:
               return

           # Apply smoothing to reduce velocity spikes
           # This is a simplified implementation
           pass

       def setup_joint_controllers(self):
           """Setup publishers for joint controllers if they don't exist"""
           # This would typically be called once at startup
           # For each joint, create a command publisher
           pass

       def run(self):
           rospy.loginfo("Physics corrector started")
           while not rospy.is_shutdown():
               if self.balance_score < self.balance_threshold:
                   self.apply_balance_correction()

               self.smooth_velocities()

               self.rate.sleep()

   if __name__ == '__main__':
       corrector = PhysicsCorrector()
       corrector.run()
   ```

3. **Execute Physics Troubleshooting**:
   ```bash
   # Run the physics diagnostics
   rosrun your_robot_diagnostics physics_diagnostics.py _max_joint_velocity:=3.0 _balance_threshold:=0.15

   # Run the physics corrector
   rosrun your_robot_diagnostics physics_corrector.py _balance_correction:=true _joint_limit_protection:=true
   ```

**Expected Outcome**: Improved humanoid robot stability in simulation with reduced physics issues.

**Learning Points**:
- Understanding physics simulation challenges in humanoid robots
- Learning to diagnose and correct joint and stability issues
- Recognizing the importance of proper physics parameters for realistic simulation

## Troubleshooting Tools and Commands

### Performance Monitoring
- **Check CPU usage**: `top` or `htop`
- **Monitor GPU**: `nvidia-smi`
- **Check memory**: `free -h`
- **Monitor ROS topics**: `rostopic hz` and `rostopic bw`

### Synchronization Checks
- **Validate TF tree**: `rosrun tf tf_monitor`
- **View transforms**: `rosrun tf view_frames`
- **Check clock synchronization**: `rostopic echo /clock`

### Physics Validation
- **Check joint states**: `rostopic echo /joint_states`
- **Monitor physics**: `gz stats`
- **Validate URDF**: `check_urdf` command

## Common Solutions and Fixes

### Performance Issues
- **Problem**: Simulation running slowly?
  - **Solution**: Reduce update rates, simplify collision geometries, decrease physics complexity

### Synchronization Problems
- **Problem**: Unity visualization out of sync with Gazebo?
  - **Solution**: Verify ROS bridge configuration, adjust update rates, implement interpolation

### Physics Instability
- **Problem**: Robot falls through ground or behaves unexpectedly?
  - **Solution**: Check mass/inertia parameters, adjust physics solver settings, verify collision models