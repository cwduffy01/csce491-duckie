#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Header

class DriverNode():
    wheel_space = .098
    speed_factor = 0.5

    def __init__(self):
        rospy.init_node("teleop")
        self.pub = rospy.Publisher("/aarrgnano/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.left_pub = rospy.Publisher("/left_dir", Int16, queue_size=10)
        self.right_pub = rospy.Publisher("/right_dir", Int16, queue_size=10)

        rospy.spin()

    def callback(self, msg):
        wheel_cmd = WheelsCmdStamped()

        lin_vel = msg.linear.x
        ang_vel = -msg.angular.z

        wheel_cmd.vel_left  = (lin_vel + (ang_vel * self.wheel_space / 2)) * self.speed_factor
        wheel_cmd.vel_right = (lin_vel - (ang_vel * self.wheel_space / 2)) * self.speed_factor

        if wheel_cmd.vel_left != 0:
            self.left_pub.publish(wheel_cmd.vel_left // abs(wheel_cmd.vel_left))
        if wheel_cmd.vel_right != 0:
            self.right_pub.publish(wheel_cmd.vel_right // abs(wheel_cmd.vel_right))

        h = Header()
        h.stamp = rospy.Time.now()
        wheel_cmd.header = h

        self.pub.publish(wheel_cmd)
        
if __name__ == "__main__":
    DriverNode()