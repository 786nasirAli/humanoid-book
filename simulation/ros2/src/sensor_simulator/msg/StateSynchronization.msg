# StateSynchronization.msg
# Message for synchronizing state between Gazebo and Unity environments

# Header
std_msgs/Header header

# Entity identification
string entity_name
string entity_type  # e.g., "robot", "object", "sensor"

# Position and orientation (for 3D synchronization)
geometry_msgs/Point position
geometry_msgs/Quaternion orientation

# Additional state information
float64 linear_velocity_x
float64 linear_velocity_y
float64 linear_velocity_z
float64 angular_velocity_x
float64 angular_velocity_y
float64 angular_velocity_z

# Timestamp information
builtin_interfaces/Time gazebo_timestamp
builtin_interfaces/Time unity_timestamp

# Synchronization confidence/quality indicator
float64 sync_confidence  # 0.0 to 1.0, where 1.0 is perfect sync

# Additional data for specific entity types
float64[10] additional_data  # Flexible array for sensor values, etc.
string status_message