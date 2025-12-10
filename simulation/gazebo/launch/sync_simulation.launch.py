import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_sensor_simulator = FindPackageShare('sensor_simulator')

    # Gazebo launch command
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])]
        )
    )

    # Transform synchronization node
    transform_sync_node = Node(
        package='sensor_simulator',
        executable='transform_sync',
        name='transform_sync_node',
        output='screen',
        parameters=[]
    )

    # Unity communication bridge (placeholder)
    unity_bridge_node = Node(
        package='sensor_simulator',
        executable='unity_bridge',
        name='unity_bridge_node',
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        gazebo,
        transform_sync_node,
        unity_bridge_node
    ])