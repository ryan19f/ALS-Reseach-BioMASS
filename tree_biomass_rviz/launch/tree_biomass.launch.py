from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tree_biomass_rviz',
            executable='biomass_node',
            name='biomass_node',
            output='screen'
        )
    ])
