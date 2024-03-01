from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    #print("cwd: ", os.getcwd())
    #print("${ISAAC_ROS_WS}", os.environ.get('ISAAC_ROS_WS'))

    camera_name = 'camera'
    basePath = os.getcwd()  # assume to be /workspaces/isaac_ros-dev
    paramsFP = os.path.join(basePath, 'src/teleopros2/teleopros2/configs/gscam_params.yaml')
    camInfoFP = os.path.join(basePath, 'src/teleopros2/teleopros2/configs/my_camera.ini')
    camInfoURL = 'file://' + camInfoFP

    # print("basePath: ", basePath)
    # print("paramsFP: ", paramsFP)
    # print("camInfoFP: ", camInfoFP)
    # print("camInfoURL: ", camInfoURL)

    baseFP = os.path.dirname(os.path.realpath(__file__))
    return LaunchDescription([
        Node(   # teleopros2 node
            package='teleopros2',
            executable='teleopros2_node',
            name='teleopros2',
            output='screen',
            parameters=[
                {
                'image-topic': '/' + camera_name + '/image_raw',
                }
            ],
        ),
        Node(   # minimal GStreamer (gscam2) camera node
            package='gscam2',
            executable='gscam_main',
            name='gscam_publisher',
            output='screen',
            namespace=camera_name,
            parameters=[#paramsFP,       # file w/ gscam parameters
                        {
                            'gscam_config': 'v4l2src device=/dev/video6 ! video/x-raw ! videoconvert',
                            'preroll': True,
                            'use_gst_timestamps': True,
                            'camera_name': camera_name,  # Camera Name
                            'camera_info_url': camInfoURL,  # Camera calibration information
                            'frame_id': 'my_camera_frame'
                        },],
            # Remap outputs to the correct namespace
            remappings=[
                ('/image_raw', '/' + camera_name + '/image_raw'),
                ('/camera_info', '/' + camera_name + '/camera_info'),
            ],
            
            )
        ])
