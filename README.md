
# TeleOp on ROS2 using WebRTC 
## for ROS2 alone, or via the NVidia docker setup for Jetson (Orin) Nano/x86

(Currently using ROS@2 Humble, that seems to be the currently supported version by NVidia.)

The use case is for teleoperation of a ROS2 robot, providing:
- teleop control:
  - view image from robot over internet (efficiently using WebRTC)
  - control robot, using either controls on screen of tilt on mobile devices
- targeted at the NVidia Jetson Orin Nano stack, but not absolutely required.

![TeloOp Screenshots](https://github.com/pgaston/teleopros2/assets/3617755/0f7b2586-aba4-4f4a-a859-2769d794dad7)

This is based on [aiortc](https://github.com/aiortc/aiortc); [jetbot-ros2](https://github.com/jdgalviss/jetbot-ros2) for inspiration; and with special consideration to [webrtc_ros](https://github.com/RobotWebTools/webrtc_ros), and others.

Please see the [Medium article](https://medium.com/@peter.gaston/add-teleop-to-your-ros2-robot-5b7b0a5606ce) for a richer discussion.   Basically this creates a browser based control panel leveraging [aiortc](https://github.com/aiortc/aiortc) for communication within the ROS2 environment.

## Installation

### Starting from scratch

(Update - 6/21/2025 - rebuilt both platforms -x86 and Orin Nano - from scratch.  Updated instructions.   Hint - follow NVidia instructions very carefully!)

There are two platforms you can (should) support:
- NVidia Jetson Orin Nano (or other, non tested).   Current version is for Jetpack 6.2 w/ the software performance update.
- x86 platform (Ubuntu 22) - this allows the use of Isaac Sim for software-in-the-loop simulation.

While this works in a 'standard' ROS2 Humble environment, the supported approach follows 
the NVidia suggested approach of using their Docker environment.   

1. Follow:
- [Developer Environment Setup](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)    
- Which is part of the broader... [Isaac ROS Docker Development Environment](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html)

Testing:
- Docker environment launches successfully.   Note that we customize this in the next step.
```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh
```

2. And, assuming you're using a Realsense camera - follow the instructions in [Hardware setup / sensors setup / Realsense](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/sensors/realsense_setup.html).    For other sensors, minor mods needed (probably.)

Testing:  Run this in the docker built environment.
```
realsense-viewer
```

### On to TeleOpROS2

3. Clone this repository (on the host)
```
cd ${ISAAC_ROS_WS}/src
git clone git@github.com:pgaston/TeleOpROS2.git
```

4. Copy all three of the files in the 'docker' folder to their place in isaac_ros_common - this is for customizing the [docker build process](https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment)
```
cd ${ISAAC_ROS_WS}/src/
mv isaac_ros_common/scripts/.isaac_ros_common-config scripts/.isaac_ros_common-config-Realsense-copy
cp TeleOpROS2/docker/.isaac_ros_common-config isaac_ros_common/scripts/
cp TeleOpROS2/docker/.isaac_ros_dev-dockerargs isaac_ros_common/scripts/
cp TeleOpROS2/docker/Dockerfile.teleopros2 isaac_ros_common/docker/
```
after this, run_dev.sh should work for our world.   This is how to run the 'standard' NVidia docker environment, with my additions, per #2 above.
```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh
```

Testing:   docker build works!

and, from inside the docker, do a full build.
```
cd /workspaces/isaac_ros-dev
colcon build
```


5. Additional requirements include:

For the Jetson Orin Nano, run the following, outside the docker environment.    This allows access to the hardware of the robot. (Your situation may vary.)
```
sudo chmod +777 /dev/gpiochip0 /dev/gpiochip1 /dev/i2c-0 /dev/i2c-1 /dev/i2c-7
```

- In your /workspaces/isaac_ros-dev/src directory git clone both [gscam](https://github.com/clydemcqueen/gscam2/tree/main) and [ros2_shared](https://github.com/ptrmu/ros2_shared)
```
cd ${ISAAC_ROS_WS}/src
git clone git@github.com:clydemcqueen/gscam2.git
git clone git@github.com:ptrmu/ros2_shared.git
```

and, from inside the docker, do a full build.
```
cd /workspaces/isaac_ros-dev
colcon build
```

6. Add SSL certificates.   This is required for mobile.   This is the default.    To change the default set the 'ssl' parameter to false.
 
- Create a 'certs' directory at the top level of teleopros2
- Create a local server.certs and server.key file in this directory.   [Here is one approach.](https://deliciousbrains.com/ssl-certificate-authority-for-local-https-development/#how-it-works) .    *Tip - don't add a passphrase.*
- (btw, I included a certs.zip that you can expand into a certs folder there.    Not secure in the slightest - but you can use to test mobile/twisting.   I can't include otherwise as it sets off a github security alert.)

*Note - your browser will show this as insecure.*   Go to advanced / proceed anyway.   Exercise for the user to do this 'correctly'.   Just pay - or search for how to get for free...


7. Build

Build (in the docker) - use symlink generally from here on out - this lets the src python code be used - greatly speeds up debugging...
```
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select teleopros2
```
Run to test.   Once you click `Connect` the server will send video from the ROS2 source to the
browser.   The image shown in this case will indicate it is not receiving ROS2 image messages, since you aren't providing any, yet.


8. Run/test with realsense camera
```
source install/setup.bash
ros2 run teleopros2 teleopros2_node
```

You can then browse to the following page with your browser:

http://127.0.0.1:8080
or, for the secure version (default)
https://127.0.0.1:8080
https://localhost:8080


If you have a realsense installed, run in another terminal (do the run_dev.sh thing to get into the same docker)
```
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```
or, in a single launch file
```
ros2 launch teleopros2 teleRSCamera_launch.py
```

** voila - WebRTC showing your realsense image **

9. Run/test with NVidia Isaac sim.   [Web page](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_isaac_sim.html)


Start Isaac sim [per directions](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_isaac_sim.html) .    BTW, I need to go to http://localhost:3080/ to restart all services (e.g., Nucleus).

```
./python.sh ${ISAAC_ROS_WS}/src/isaac_ros_nvblox/nvblox_examples/nvblox_isaac_sim/omniverse_scripts/start_isaac_sim.py --gpu_physics_enabled
```

This Isaac sim publishes the image on (at least my version)
```/image_raw```
so you may need to change the teleopros2.py code to subscribe to that (okay, this can be fixed so it's easier...)

to move the robot for testing purposes, a simple node is 'teleop_twist_keyboard'.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
I also have a customization of the 'simple room' with a jetbot.   This publishes images and accepts robot movement commands.

7. Run/test with a GStreamer sourced camera, say a webcam or a CSI camera on the Jetson Orin Nano.

Adjust your 'gscam_config' string in launch/teleGSCamera_lannch.py.   The current default is to use /dev/video6.

To run
```
ros2 launch teleopros2 teleGSCamera_launch.py
```
and again, on your browser go to https://localhost:8080/

## For Jetbot...

8. Now for a Jetbot!
(ROS - but interesting github - https://github.com/issaiass/jetbot_diff_drive)




## To do's (potentially)
- Performance enhancement.   Use gstreamer to also create a h.264 compressed stream
that can be used instead of the image topic.   This will utilize the GPU most effectively.   See [discussion](https://github.com/aiortc/aiortc/discussions/769)
- Use on a real Jetson Orin Nano robot.   Turns out the original Nano is too old for the new NVidia docker setup, and the Orin Nano doesn't fit on, say the Jetbot.  (Different power needs, ...)   Still building...

## Useful links

- [Isaac ROS Common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html) 

## Other links
-[linorobot project](https://github.com/linorobot/linorobot2)
-[jetbot-ros2 project](https://github.com/jdgalviss/jetbot-ros2)





https://control.ros.org/humble/doc/getting_started/getting_started.html
https://github.com/ros-controls/




clone humble branch
follow instructions - https://control.ros.org/master/doc/ros2_control_demos/doc/index.html#build-from-debian-packages


VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV

************************************************************************
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
  ./scripts/run_dev.sh
************************************************************************

==> 

rm -rf install/
rm -rf build/





# rm -rf src/ros2_control_demos
# ackermann_msgs


==> (to verify it all builds, then run example 1 to ensure it 'works')
     https://github.com/ros-controls/ros2_control_demos/blob/master/example_1/doc/userdoc.rst

rm -rf install/
rm -rf build/

. /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --symlink-install

==> run an example
  tab 1:

source install/setup.bash
ros2 launch ros2_control_demo_example_1 view_robot.launch.py
  tab 2:
source /opt/ros/${ROS_DISTRO}/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui




--> one or both...

git clone https://github.com/ros-controls/ros2_control_demos -b humble

# for now, throws error - missing file 
# #include "hardware_interface/lexical_casts.hpp"
# so deleted examples 2, 8, 14 
# seems like the apt install sources haven't caught up yet w/ the github code
# --> should fix itself over time

# for build from source - btw, not working yet...
git clone https://github.com/ros-controls/ros2_control.git -b humble
git clone https://github.com/ros-controls/ros2_controllers.git -b humble
git clone git@github.com:ros-controls/control_msgs.git -b humble
git clone git@github.com:ros-controls/realtime_tools.git


cd ..
rosdep update --rosdistro=$ROS_DISTRO
sudo apt update
sudo apt upgrade
rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}

# must be some way to force building these first...

. /opt/ros/${ROS_DISTRO}/setup.sh

# NO - install from source
# sudo apt install ros-humble-control-msgs

==>

colcon build --packages-select realtime_tools
colcon build --packages-select control_msgs
colcon build --packages-select ros2_control_test_assets
colcon build --packages-select controller_manager_msgs
colcon build --packages-select joint_limits
colcon build --packages-select rqt_controller_manager
colcon build --packages-select hardware_interface
colcon build --packages-select hardware_interface_testing
colcon build --packages-select controller_interface 
colcon build --packages-select controller_manager
colcon build --packages-select transmission_interface 
colcon build --packages-select ros2controlcli 
colcon build --packages-select ros2_control




colcon build --symlink-install



source install/setup.bash
ros2 launch ros2_control_demo_example_2 view_robot.launch.py

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
colcon build --symlink-install
[1.807s] WARNING:colcon.colcon_core.package_selection:Some selected packages are already built in one or more underlay workspaces:
  'controller_interface' is in: /workspaces/isaac_ros-dev/install/controller_interface, /opt/ros/humble
  'hardware_interface' is in: /workspaces/isaac_ros-dev/install/hardware_interface, /opt/ros/humble
  'controller_manager' is in: /workspaces/isaac_ros-dev/install/controller_manager
  'ros2_control_test_assets' is in: /workspaces/isaac_ros-dev/install/ros2_control_test_assets
If a package in a merged underlay workspace is overridden and it installs headers, then all packages in the overlay must sort their include directories by workspace order. Failure to do so may result in build failures or undefined behavior at run time.
If the overridden package is used by another package in any underlay, then the overriding package in the overlay must be API and ABI compatible or undefined behavior at run time may occur.

If you understand the risks and want to override a package anyways, add the following to the command line:
  --allow-overriding controller_interface controller_manager hardware_interface ros2_control_test_assets


colcon build --packages-select controller_manager_msgs joint_limits controller_interface transmission_interface controller_manager ros2controlcli

- controller_manager_msgs
- joint_limits
- controller_interface
- transmission_interface
- controller_manager
- ros2controlcli


# Getting an ssl cert

Current path:
- use Apache
- add in certbot
- get ssl cert
- move to teleopros2 folder
- !!

https://certbot.eff.org/
https://letsencrypt.org/





