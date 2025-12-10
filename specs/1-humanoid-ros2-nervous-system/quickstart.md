# Quickstart Guide: Setting Up ROS 2 for Humanoid Robotics

## Prerequisites

Before starting with this module, ensure you have:

1. **ROS 2 Humble Hawksbill** installed on your system
   - Follow the official installation guide for your OS: https://docs.ros.org/en/humble/Installation.html
   - Install the desktop version which includes development tools

2. **Python 3.8+** with pip package manager

3. **Basic Python knowledge** (variables, functions, classes)

## Environment Setup

1. **Source ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
   
   For convenience, add this to your `.bashrc`:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

2. **Create a workspace** for your humanoid robot project:
   ```bash
   mkdir -p ~/humanoid_ws/src
   cd ~/humanoid_ws
   ```

3. **Build the workspace**:
   ```bash
   colcon build
   source install/setup.bash
   ```

## First ROS 2 Node

Let's create your first ROS 2 node that simulates a simple sensor publisher:

1. **Create a new package**:
   ```bash
   cd ~/humanoid_ws/src
   ros2 pkg create --build-type ament_python sensor_publisher_py
   ```

2. **Navigate to the package directory**:
   ```bash
   cd sensor_publisher_py
   ```

3. **Create the publisher code** in `sensor_publisher_py/sensor_publisher.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class SensorPublisher(Node):
       def __init__(self):
           super().__init__('sensor_publisher')
           self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = f'Sensor reading: {self.i}'
           self.publisher_.publish(msg)
           self.get_logger().info(f'Publishing: "{msg.data}"')
           self.i += 1

   def main(args=None):
       rclpy.init(args=args)
       sensor_publisher = SensorPublisher()
       rclpy.spin(sensor_publisher)
       sensor_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. **Update setup.py** to make the node executable:
   In `setup.py`, add to the `entry_points` section under `console_scripts`:
   ```python
   'sensor_publisher = sensor_publisher_py.sensor_publisher:main'
   ```

5. **Build and run the node**:
   ```bash
   cd ~/humanoid_ws
   colcon build --packages-select sensor_publisher_py
   source install/setup.bash
   ros2 run sensor_publisher_py sensor_publisher
   ```

## Testing Communication

In a new terminal (while the publisher is running), create a subscriber to test communication:

1. **Create subscriber node** in `sensor_publisher_py/sensor_subscriber.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class SensorSubscriber(Node):
       def __init__(self):
           super().__init__('sensor_subscriber')
           self.subscription = self.create_subscription(
               String,
               'sensor_data',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info(f'I heard: "{msg.data}"')

   def main(args=None):
       rclpy.init(args=args)
       sensor_subscriber = SensorSubscriber()
       rclpy.spin(sensor_subscriber)
       sensor_subscriber.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. **Update setup.py** to add the subscriber:
   ```python
   'sensor_subscriber = sensor_publisher_py.sensor_subscriber:main'
   ```

3. **Build and run both nodes**:
   Terminal 1 (publisher):
   ```bash
   cd ~/humanoid_ws
   source install/setup.bash
   ros2 run sensor_publisher_py sensor_publisher
   ```

   Terminal 2 (subscriber):
   ```bash
   cd ~/humanoid_ws
   source install/setup.bash
   ros2 run sensor_publisher_py sensor_subscriber
   ```

You should now see the publisher sending messages and the subscriber receiving them. This demonstrates the basic publisher-subscriber communication pattern that serves as the "nervous system" for humanoid robots.

## Next Steps

After completing this quickstart:
1. Explore the ROS 2 command-line tools: `ros2 topic`, `ros2 node`, `ros2 service`
2. Learn about ROS 2 message types in `/opt/ros/humble/share/std_msgs/msg/`
3. Continue with the next sections in Module 1 to understand more complex communication patterns