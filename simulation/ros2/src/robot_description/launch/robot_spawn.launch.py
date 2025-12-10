import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('robot_description')
    
    # Path to the robot URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'basic_robot.urdf')
    
    # Launch Gazebo with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/gazebo.launch.py']
        ),
        launch_arguments={
            'world': os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world')
        }.items()
    )
    
    # Launch robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )
    
    # Launch spawn entity node to add the robot to Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'basic_robot',
            '-file', urdf_file
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])