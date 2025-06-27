from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleopros2',
            executable='teleopros2_node',
            name='teleopros2',
            output='screen',
            parameters=[],
        ),
    ])