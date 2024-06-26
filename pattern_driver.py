import rospy
from geometry_msgs.msg import Twist, Pose2D
from math import *

class Point():
    x = 0.0
    y = 0.0

    def __init__(self, px, py):
        self.x = px
        self.y = py

class PatternDriver():

    def __init__(self):
        self.posx = 0
        self.posy = 0
        self.theta = 0

        self.test_sleep = 3

        self.sleep = True
        self.sleep_t = 1

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
        self.reset_pub = rospy.Publisher("/set_pose", Pose2D, queue_size=1)
        rospy.Subscriber("/duckie_pose", Pose2D, self.callback)

        rospy.init_node("duckie_driver")

        msg = Pose2D()
        msg.x = 0
        msg.y = 0
        msg.theta = 0

        self.reset_pub.publish(msg)
        rospy.sleep(1)

    def callback(self, msg):
        self.posx = msg.x
        self.posy = msg.y
        self.theta = msg.theta

    def test_sleep(self, t):
        if not rospy.is_shutdown():
            rospy.sleep(t)

    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = 0
        self.pub.publish(msg)

    def move(self, distance, lin_vel = 0.5, ang_vel = 0.0):
        total_dist = 0.0
        last_posx = self.posx
        last_posy = self.posy

        msg = Twist()
        msg.linear.x = lin_vel * (abs(distance) / distance)
        msg.angular.z = ang_vel
        self.pub.publish(msg)

        while (total_dist < abs(distance)) and not rospy.is_shutdown():
            dx = ((self.posx - last_posx)**2 + (self.posy - last_posy)**2)**0.5
            total_dist += dx
            last_posx = self.posx
            last_posy = self.posy
            rospy.sleep(0.01)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

        if self.sleep:
            self.check_sleep(self.sleep_t)

    def set_theta(self, angle, ang_vel = 0.75, lin_vel = 0.0, sleep = True, sleep_t = 1):
        msg = Twist()

        start_angle = self.theta
        angle_diff = angle - start_angle 
        direction = (abs(angle_diff) / angle_diff)
        angle_diff = abs(angle_diff)


        msg.angular.z = ang_vel * direction

        self.pub.publish(msg)
        while (abs(angle - self.theta) <= angle_diff) and not rospy.is_shutdown():
            angle_diff = abs(angle - self.theta)
            rospy.sleep(0.01)

        msg.angular.z = 0.0
        self.pub.publish(msg)

        if self.sleep:
            self.check_sleep(self.sleep_t)


    def drive_square(self, side_length = 0.5):
        adjust_w = 0.5
        ang_vel = 0.75
        pd.move(side_length, ang_vel=0.5)
        pd.set_theta(pi/2, ang_vel)
        pd.move(side_length, ang_vel=-0.2)
        pd.set_theta(pi, ang_vel)
        pd.move(side_length, ang_vel=-0.5)
        pd.set_theta(3 * pi / 2, ang_vel)
        pd.move(side_length, ang_vel=0.0)
        pd.set_theta(0, ang_vel)
    
    def check_sleep(self, duration):
        if not rospy.is_shutdown():
            rospy.sleep(duration)

if __name__ == "__main__":
    pd = PatternDriver()

    pd.stop()


