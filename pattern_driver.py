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

        # rospy.spin()

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

    def move(self, distance, lin_vel = 0.5, sleep = True, sleep_t = 1):
        start_x = self.posx
        start_y = self.posy

        msg = Twist()
        msg.linear.x = lin_vel * (abs(distance) / distance)
        self.pub.publish(msg)

        while (((self.posx - start_x)**2 + (self.posy - start_y)**2)**0.5 < abs(distance)) and not rospy.is_shutdown():
            # print("distance: " + str(((self.posx - start_x)**2 + (self.posy - start_y)**2)**0.5))
            rospy.sleep(0.1)

        msg.linear.x = 0.0
        self.pub.publish(msg)

        if sleep:
            self.check_sleep(sleep_t)

    def set_theta(self, angle, ang_vel = 0.5, lin_vel = 0.0, sleep = True, sleep_t = 1):
        msg = Twist()

        start_angle = self.theta
        angle_diff = angle - start_angle    
        # print(angle_diff)
        msg.angular.z = ang_vel * (abs(angle_diff) / angle_diff)
        angle_diff = abs(angle_diff)

        self.pub.publish(msg)
        while (abs(angle - self.theta) <= angle_diff) and not rospy.is_shutdown():
            # print("theta: " + str(self.theta))
            angle_diff = abs(abs(angle - self.theta))
            rospy.sleep(0.1)

        msg.angular.z = 0.0
        self.pub.publish(msg)

        if sleep:
            self.check_sleep(sleep_t)


    def drive_square(self, side_length = 0.5):
        pd.move(side_length)
        pd.set_theta(pi/2)
        pd.move(side_length)
        pd.set_theta(pi)
        pd.move(side_length)
        pd.set_theta(3 * pi / 2)
        pd.move(side_length)
        pd.set_theta(2 * pi)
    
    def check_sleep(self, duration):
        if not rospy.is_shutdown():
            rospy.sleep(duration)

if __name__ == "__main__":
    pd = PatternDriver()

    pd.drive_square()

    # ang_vel >= 0.5
    # lin_vel >= 0.1


