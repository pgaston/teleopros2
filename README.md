# TeleOp on ROS2 using WebRTC 
## for ROS2 alone, or via the NVidia docker setup for Jetson (Orin) Nano/x86

The use case is for teleoperation of a ROS2 robot, providing:
- teleop desktop - ready for further customization - it already provides tilt control on mobile devices
- efficient transport using WebRTC - both video and data
- generally targeted at the NVidia stack, but not required.

![Tele Op Screen shots](iamges/TeleOpScreenshots.png "Tele Op Screen shots")

This is based on [aiortc](https://github.com/aiortc/aiortc); [jetbot-ros2](https://github.com/jdgalviss/jetbot-ros2) for inspiration; and with special consideration to [webrtc_ros](https://github.com/RobotWebTools/webrtc_ros) (which I couldn't get to work) from 'Robot Web Tools', and others.

## Overview

Please see the Medium article for a richer discussion.   Basically this creates a browser based desktop leveraging [aiortc](https://github.com/aiortc/aiortc) within the ROS2 environment.

## Installation

This works in either a 'standard' ROS2 environment or in the NVidia docker path -  [Developer Environment Setup](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)    More specifically [Isaac ROS Docker Development Environment](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html).    

1. Clone this repository (on the host)
cd ${ISAAC_ROS_WS}/src
git clone git@github.com:pgaston/TeleOpROS2.git

2. Copy the following two files - this is for customizing the docker build process
```
${ISAAC_ROS_WS}/src/TeleOpROS2/docker/.isaac_ros_common-config
${ISAAC_ROS_WS}/src/TeleOpROS2/docker/.isaac_ros_common-config
```
to the folder
```
${ISAAC_ROS_WS}/src/isaac_ros_common/scripts
```
after this, run_dev.sh should work.   This is how to run the 'standard' NVidia docker environment, with my additions, per #2 above.
```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh
```

3. Build

Build (in the docker)
```
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select pywebrtc
```
4. Run/test with realsense camera
```
source install/setup.bash
ros2 run teleopros2 teleopros2
```

You can then browse to the following page with your browser:

http://127.0.0.1:8080
or, for the secure version
https://127.0.0.1:8080

Once you click `Connect` the server will send video from the ROS2 source to the
browser.   The initial screen is gray.

If you have a realsense installed, run in another terminal (do the run_dev.sh thing to get into the same docker)
```
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

** voila - WebRTC showing your realsense image **

5. Run/test with NVidia Isaac sim.   [Web page](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_isaac_sim.html)



Start Isaac sim [per directions](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_isaac_sim.html) .    BTW, I need to go to http://localhost:3080/ to restart all services (e.g., Nucleus).

```
./python.sh ${ISAAC_ROS_WS}/src/isaac_ros_nvblox/nvblox_examples/nvblox_isaac_sim/omniverse_scripts/start_isaac_sim.py --gpu_physics_enabled
```

This Isaac sim publishes the image on 
```/front/stereo_camera/left/rgb```
so you may need to change the pywebrtc.py code to subscribe to that (okay, this can be fixed so it's easier...)

to move the robot for testing purposes, a simple node is 'teleop_twist_keyboard'.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
I also have a customization of the 'simple room' with a jetbot.   This publishes images and accepts robot movement commands.



## Useful links

- [Isaac ROS Common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html) 

