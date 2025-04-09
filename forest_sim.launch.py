from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('your_package_name'),  # Replace with your package
        'worlds',
        'forest_world.sdf'
    )

    return LaunchDescription([
        # Launch Gazebo with forest world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Normalization Node
        Node(
            package='your_package_name',  # Replace with your package name
            executable='normalize_gps_node',
            name='normalize_gps_node',
            output='screen'
        ),
    ])
