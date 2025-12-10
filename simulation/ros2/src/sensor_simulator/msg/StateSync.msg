# StateSync.msg
# Message for synchronizing state between Gazebo and Unity

# Header
std_msgs/Header header

# Pose information
geometry_msgs/Pose pose

# Twist information (velocity)
geometry_msgs/Twist twist

# Timestamp for synchronization
builtin_interfaces/Time sync_time

# Additional state data as needed
float64[] joint_states
string[] joint_names