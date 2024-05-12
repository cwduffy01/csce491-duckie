#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Header

class DriverNode():
    wheel_space = 80e-3
    speed_factor = 0.5

    vl = 0
    vr = 0

    def __init__(self):
        rospy.init_node("teleop")
        self.pub = rospy.Publisher("/aarrgnano/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=100)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)

        rospy.Subscriber("/enc_left_vel", Float32, self.left_update)
        rospy.Subscriber("/enc_right_vel", Float32, self.right_update)

        rospy.spin()

    def left_update(self, msg):
        self.vl = msg.data

    def right_update(self, msg):
        self.vr = msg.data

    # translate twist messages to duckiebot drive commands
    def callback(self, msg):
        wheel_cmd = WheelsCmdStamped()

        lin_vel = msg.linear.x
        ang_vel = -msg.angular.z

        wheel_cmd.vel_left  = (lin_vel + (ang_vel * self.wheel_space / 2)) * self.speed_factor
        wheel_cmd.vel_right = (lin_vel - (ang_vel * self.wheel_space / 2)) * self.speed_factor

        h = Header()
        h.stamp = rospy.Time.now()
        wheel_cmd.header = h

        self.pub.publish(wheel_cmd)

        if wheel_cmd.vel_left == 0 and wheel_cmd.vel_right == 0:
            while self.vl != 0 or self.vr != 0:
                self.pub.publish(wheel_cmd)
                rospy.sleep(0.01)
        else:
            while abs(self.vl) < abs(wheel_cmd.vel_left) or abs(self.vr) < abs(wheel_cmd.vel_right):
                self.pub.publish(wheel_cmd)
                rospy.sleep(0.01)
        
if __name__ == "__main__":
    DriverNode()