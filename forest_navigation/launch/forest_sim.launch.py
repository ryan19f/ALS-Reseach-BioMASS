from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='gazebo_ros', executable='gzserver', output='screen', arguments=['-s', 'libgazebo_ros_factory.so']),
        Node(package='forest_navigation', executable='gps_normalizer', output='screen'),
        Node(package='forest_navigation', executable='lidar_filter_node', output='screen'),
        Node(package='forest_navigation', executable='imu_filter_node', output='screen'),
    ])
