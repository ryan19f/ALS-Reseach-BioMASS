from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tree_biomass_rviz',
            executable='biomass_visualizer_node',
            name='biomass_visualizer_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', os.path.join(
                os.getenv('HOME'),
                'space_robotics_project/ros2_ws/src/tree_biomass_rviz/config/biomass_view.rviz'
            )],
            output='screen'
        )
    ])
