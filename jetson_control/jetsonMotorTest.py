'''
Jetson motor test
- just see what we can get to work
'''

import atexit

from adafruit_motorkit import MotorKit


## ROS2 Node to publish stats
# WebRTC node publish/subscribe
class MoveRobot():

    def __init__(self):

        # setup Motor control via I2C
        self.kit = MotorKit()
        self.kit.motor1.throttle = 0.0
        self.kit.motor2.throttle = 0.0

        # parameters

    def stopRobot(self):
        print("Stopping robot")
        self.kit.motor1.throttle = 0.0
        self.kit.motor2.throttle = 0.0
        pass

    def pwr_to_throttle(self, pwr):
        # pwr is -100 to +100
        pwr = max(-100, min(100, pwr))
        return pwr / 100.0

    # values are from -100 to +100
    def twist_callback(self, cmd_vel_msg):
        x = cmd_vel_msg.linear.x        # Forward/Back
        # y = cmd_vel_msg.linear.y
        # z = cmd_vel_msg.linear.z
        # rx = cmd_vel_msg.angular.x
        # ry = cmd_vel_msg.angular.y
        rz = cmd_vel_msg.angular.z      # Rotation

        pwr = x / 100.0
        rot = rz / 100.0

        pwr = max(-1.0, min(1.0, pwr))      # just in case commmands were off...
        rot = max(-1.0, min(1.0, rot))

        # to do this 'right' - https://answers.ros.org/question/244540/kinematic-and-dynamic-equations-of-robot/
        # if we were doing this for RPMs...
        # vel_l = ((cmd_vel_msg.linear.x - (cmd_vel_msg.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)
        # vel_r = ((cmd_vel_msg.linear.x + (cmd_vel_msg.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)

        lt = pwr - rot
        rt = pwr + rot

        lt = max(-1.0, min(1.0, lt))
        rt = max(-1.0, min(1.0, rt))

        print("Power: ", pwr, " Rotation: ", rot)
        print("Left: ", lt, " Right: ", rt)

        self.kit.motor1.throttle = rt
        self.kit.motor2.throttle = lt
        
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

    MoveNode = MoveRobot()     # do this sequentially, as it captures the parameters
    atexit.register(MoveNode.stopRobot)        # just in case...

    print("Starting motor test...")


    print("Running test commands...")


    print("Test complete, stopping robot")
    MoveNode.stopRobot()        # hmmm, wonder which way to stop robot is more certain?


if __name__ == "__main__":
    main()


