from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(   # teleopros2 node
            package='teleopros2',
            executable='teleopros2_node',
            name='teleopros2',
            # parameters=[...],
        ),
        Node(   # minimal realsense camera node
            package='realsense2_camera',
            executable='realsense2_camera_node',
            arguments=['--ros-args',
                       '-p', 'rgb_camera.profile:=640x480x30',  # 640x480 at 30fps
                       ],
            )
        ])
