# ROS 2 Examples for Humanoid AI Book

This directory contains example code for Module 1: The Robotic Nervous System.

## Running Examples

To run these examples, you need to have ROS 2 Humble Hawksbill installed with Python support.

### Publisher Example

1. Open a terminal and source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Navigate to this directory and make the script executable:
   ```bash
   cd content/examples/publisher
   chmod +x simple_publisher.py
   ```

3. Run the publisher:
   ```bash
   python3 simple_publisher.py
   ```

### Subscriber Example

1. In another terminal, source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Navigate to the subscriber directory:
   ```bash
   cd content/examples/subscriber
   ```

3. Run the subscriber:
   ```bash
   python3 simple_subscriber.py
   ```

4. You should see the publisher sending messages and the subscriber receiving them.

### Running Both Together

For best results, run the subscriber first in one terminal, then run the publisher in another terminal.

## Directory Structure

- `publisher/`: Examples of ROS 2 publishers
- `subscriber/`: Examples of ROS 2 subscribers
- `controller/`: Examples of ROS 2 controllers for humanoid robots
- `urdf/`: URDF examples for humanoid robot structures
- `integration/`: Complete examples integrating multiple concepts

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Python 3.8+
- rclpy library (comes with ROS 2 installation)

## Troubleshooting

If you encounter issues running these examples, please refer to the troubleshooting guide:
- Location: `content/module-1/troubleshooting.md`
- Common issues covered: installation problems, communication issues, performance issues, and humanoid-specific problems