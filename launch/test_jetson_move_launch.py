from launch import LaunchDescription
from launch_ros.actions import Node
import os
import platform

# to support testing of the jetson motors...

# For variations between x86 and Jetson
arch = platform.machine()
if arch.startswith('arm') or arch.startswith('aarch64'):
    processor_type = 'arm'
    bJetson = True
else:
    processor_type = 'x86'
    bJetson = False

if processor_type != 'arm':
    print("jetson not detected - not supported, duh")
    exit()


print("Detected processor type: {processor_type}")

def generate_launch_description():

    return LaunchDescription([
        # web site, listens for camera, generates move topics, etc.
        Node(
            package='teleopros2',
            executable='jetson_move_node',
            name='test_jetson_move',
            output='screen',
            parameters=[],
        ),
    ])
