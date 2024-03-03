from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import os

# Pipeline for CSI camera on Jetson Orin Nano
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=640,
    display_height=480,
    framerate=24,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! videoconvert"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def generate_launch_description():
    camera_name = 'camera'
    basePath = os.getcwd()  # assume to be /workspaces/isaac_ros-dev
    paramsFP = os.path.join(basePath, 'src/teleopros2/teleopros2/configs/gscam_params.yaml')
    camInfoFP = os.path.join(basePath, 'src/teleopros2/teleopros2/configs/my_camera.ini')
    camInfoURL = 'file://' + camInfoFP

    # CSI camera pipeline on Jetson Orin Nano in NVidia docker (8MP resolution)
    gpipeline = gstreamer_pipeline()
    #gpipeline = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! videoconvert"
    print("GStreamer pipeline: ", gpipeline)

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
                            # jetson orin nano - CSI camera (8MP resolution)
                            'gscam_config': gpipeline,
                            # webcam on video6
                            # 'gscam_config': 'v4l2src device=/dev/video6 ! video/x-raw ! videoconvert',
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
