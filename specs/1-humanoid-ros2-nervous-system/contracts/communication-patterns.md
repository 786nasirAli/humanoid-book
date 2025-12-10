# API Contracts: ROS 2 Communication Patterns

## Publisher-Subscriber Communication Pattern

### Endpoint: /sensor_data
**Purpose**: Transmit sensor readings from humanoid robot sensors to processing nodes
**Pattern**: Topic-based communication (one-way)
**Message Type**: sensor_msgs/msg/JointState
**QoS Profile**: default (reliable, keep_last 10 samples)
**Publisher**: Sensor Interface Node
**Subscribers**: State Estimation, Control Algorithm, Monitoring

### Endpoint: /motor_commands
**Purpose**: Send motor commands from controllers to actuator interfaces
**Pattern**: Topic-based communication (one-way)
**Message Type**: sensor_msgs/msg/JointState
**QoS Profile**: default (reliable, keep_last 10 samples)
**Publisher**: Controller Node
**Subscribers**: Actuator Interface

### Endpoint: /robot_state
**Purpose**: Broadcast current state of the humanoid robot
**Pattern**: Topic-based communication (one-way)
**Message Type**: nav_msgs/msg/Odometry
**QoS Profile**: default (reliable, keep_last 10 samples)
**Publisher**: State Estimation Node
**Subscribers**: Visualization, Monitoring, Logging

## Service-Based Communication Pattern

### Endpoint: /get_robot_status
**Purpose**: Request current operational status of the humanoid robot
**Pattern**: Request-reply service
**Service Type**: std_srvs/srv/Trigger
**Request Message**: Empty
**Response Message**: 
  - success: bool (whether request succeeded)
  - message: string (status description)
**Server**: Status Manager Node
**Clients**: Monitoring Interface, Debugging Tools

### Endpoint: /set_control_mode
**Purpose**: Request to change the control mode of the humanoid robot
**Pattern**: Request-reply service
**Service Type**: custom (control_msgs/srv/SetControlMode)
**Request Message**: 
  - mode: string (desired control mode)
**Response Message**: 
  - success: bool (whether mode was set successfully)
  - message: string (result description)
**Server**: Control Manager Node
**Clients**: Operator Interface, Higher-level Planner

## Action-Based Communication Pattern (Advanced)

### Endpoint: /execute_trajectory
**Purpose**: Send trajectory for the humanoid robot to follow with feedback
**Pattern**: Action-based communication (goal-feedback-result)
**Action Type**: control_msgs/action/FollowJointTrajectory
**Goal Message**: 
  - trajectory: trajectory_msgs/msg/JointTrajectory
  - goal_time_tolerance: builtin_interfaces/msg/Duration
**Feedback Message**: 
  - joint_names: string[]
  - desired: trajectory_msgs/msg/JointTrajectoryPoint
  - actual: trajectory_msgs/msg/JointTrajectoryPoint
  - error: trajectory_msgs/msg/JointTrajectoryPoint
**Result Message**: 
  - error_code: int32
  - error_string: string
**Server**: Trajectory Controller Node
**Clients**: Motion Planning, Operator Interface