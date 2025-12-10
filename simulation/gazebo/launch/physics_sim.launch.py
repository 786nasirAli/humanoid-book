import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='basic_physics.world')

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='basic_physics.world',
        description='Choose one of the world files from `/simulation/gazebo/worlds`'
    )

    # Get the package share directory
    pkg_share = get_package_share_directory('robot_description')
    
    # Include the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(get_package_share_directory('robot_description'), '..', '..', '..', 'simulation', 'gazebo', 'worlds', 'basic_physics.world'),
            'use_sim_time': use_sim_time
        }.items()
    )

    # Node to spawn the collision test model
    spawn_collision_test = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'collision_test',
            '-file', os.path.join(get_package_share_directory('gazebo_ros'), 'models', 'collision_test.sdf'),
            '-x', '2.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Node to spawn the basic robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'basic_robot',
            '-file', os.path.join(pkg_share, 'urdf', 'basic_robot.urdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Launch description
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_world_cmd)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_collision_test)
    ld.add_action(spawn_robot)

    return ld