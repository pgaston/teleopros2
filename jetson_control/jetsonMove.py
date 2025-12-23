'''
Jetbot move
- The ROS2 node diff_drive_controller is doing the actual work to translate request into motor inputs...
- subscribe to cmd_vel_out
- move robot
'''

import atexit




# ros2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from PCA9685ServoESC import SteeringDriveMotorController


# ROS2 move to actually move jetson motors
class MoveRobot(Node):

    def __init__(self):
        super().__init__('moverobot')

        # setup Motor control via I2C
        self.kit = SteeringDriveMotorController()

        # parameters
        self.declare_parameter('twist-topic', 'cmd_vel')
        argstwisttopic = self.get_parameter('twist-topic').value

        self.subscription = self.create_subscription(Twist,argstwisttopic,self.twist_callback,10)
        self.subscription  # prevent unused variable warning

    def stopRobot(self):
        print("Stopping robot")
        self.kit.done()

    # values are from -100 to +100
    def twist_callback(self, cmd_vel_msg):
        x = cmd_vel_msg.linear.x        # Forward/Back
        # y = cmd_vel_msg.linear.y
        # z = cmd_vel_msg.linear.z
        # rx = cmd_vel_msg.angular.x
        # ry = cmd_vel_msg.angular.y
        rz = cmd_vel_msg.angular.z      # Rotation

        pwr = x 
        rot = rz

        # pwr = x / 100.0
        # rot = rz / 100.0

        pwr = max(-1.0, min(1.0, pwr))      # just in case commmands were off...
        rot = max(-1.0, min(1.0, rot))

        # ackerman drive
        print("Ackerman Drive: Power: ", pwr, " Rotation: ", rot)
        self.kit.set_SteerDrive(rot,pwr)

        # jetbot
        # to do this 'right' - https://answers.ros.org/question/244540/kinematic-and-dynamic-equations-of-robot/
        # if we were doing this for RPMs...
        # vel_l = ((cmd_vel_msg.linear.x - (cmd_vel_msg.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)
        # vel_r = ((cmd_vel_msg.linear.x + (cmd_vel_msg.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)

        # lt = pwr - rot
        # rt = pwr + rot

        # lt = max(-1.0, min(1.0, lt))
        # rt = max(-1.0, min(1.0, rt))

        # print("Power: ", pwr, " Rotation: ", rot)
        # print("Left: ", lt, " Right: ", rt)

        # self.kit.set_SteerDrive(rot,pwr)

        
'''
# forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
# back
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
# stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once


# rotate right
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 20.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 100.0}}" --once
# rotate left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 20.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -100.0}}" --once
# rotate right nothing
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 100.0}}" --once


'''


def main():

    rclpy.init()
    MoveNode = MoveRobot()     # do this sequentially, as it captures the parameters
    atexit.register(MoveNode.stopRobot)        # just in case...

    rclpy.spin(MoveNode)

    MoveNode.stopRobot()        # hmmm, wonder which way to stop robot is more certain?
    rclpy.shutdown()

if __name__ == "__main__":
    main()


