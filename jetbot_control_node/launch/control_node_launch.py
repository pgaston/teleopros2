from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetbot_control_node',
            executable='control_node',
            name='jetbot_control_node',
            output='screen',
            parameters=[{'param_name': 'param_value'}],  # Replace with actual parameters
            remappings=[('/old/topic', '/new/topic')]  # Replace with actual topic remappings
        )
    ])