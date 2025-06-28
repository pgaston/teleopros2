from launch import LaunchDescription
from launch_ros.actions import Node
import os
import platform

# For variations between x86 and Jetson
arch = platform.machine()
if arch.startswith('arm') or arch.startswith('aarch64'):
    processor_type = 'arm'
    bJetson = True
else:
    processor_type = 'x86'
    bJetson = False

print(f"Detected processor type: {processor_type}")


HOST_IP = os.getenv('HOST_IP', "0.0.0.0")

def generate_launch_description():

    return LaunchDescription([
        # to do - add jetbotMove node here
        # web site, listens for camera, generates move topics, etc.
        Node(
            package='teleopros2',
            executable='teleopros2_node',
            name='teleopros2',
            output='screen',
            parameters=[
                {'twist-topic': 'cmd_vel'},
                {'json-topic': 'teleoppub'},
                {'image-topic': '/color/image_raw'},
                # {'image-topic': '/camera/color/image_raw'},
                {'ssl': True},
                {'cert-file': 'certs/server.cert'},
                {'key-file': 'certs/server.key'},
                {'host': HOST_IP}, 
                {'port': 8080},
                {'fps': 15},
                {'verbose': False}
            ],
        ),
        # Realsense camera node
        Node(   
            package='realsense2_camera',
            executable='realsense2_camera_node',
            arguments=['--ros-args',
                       '-p', 'rgb_camera.profile:=640x480x30',  # 640x480 at 30fps
                       ],
            )
    ])