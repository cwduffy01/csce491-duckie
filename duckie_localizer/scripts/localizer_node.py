#!/usr/bin/env python
import rospy
from math import *
import numpy as np
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32

class Localizer:
    ticks_per_rev = 137.0
    wheel_diam = .066
    wheel_space = 80e-3

    right_enc = None
    left_enc = None

    radius = 0
    omega = 0

    posx = 0
    posy = 0
    theta = 0

    vr = 0
    vl = 0

    last_time = 0

    def __init__(self):
        rospy.init_node("localizer")

        rospy.Subscriber("/aarrgnano/left_wheel_encoder_node/tick", WheelEncoderStamped, self.left_callback)
        rospy.Subscriber("/aarrgnano/right_wheel_encoder_node/tick", WheelEncoderStamped, self.right_callback)
        rospy.Subscriber("/set_pose", Pose2D, self.set_pose)    # used to reset the pose of the bot during testing

        self.pose_pub = rospy.Publisher("/duckie_pose", Pose2D, queue_size=10)
        self.left_vel_pub = rospy.Publisher("/enc_left_vel", Float32, queue_size=10)
        self.right_vel_pub = rospy.Publisher("/enc_right_vel", Float32, queue_size=10)

        self.left_last_time = float(rospy.get_rostime().to_nsec())
        self.right_last_time = float(rospy.get_rostime().to_nsec())

        # update pose every 1/100 seconds
        rate = rospy.Rate(100)
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            new_time = rospy.get_time()
            self.pose_update(new_time - t0)
            t0 = new_time
            rate.sleep()

    def set_pose(self, msg):
        self.posx = msg.x
        self.posy = msg.y
        self.theta = msg.theta

    # callbacks for each wheel
    def left_callback(self, msg):
        if self.left_enc is None:
            self.left_enc = msg.data
            return
        
        now = float(msg.header.stamp.to_nsec())
        delta_time = (now - self.left_last_time) * 1e-9
        self.left_last_time = now

        delta_ticks = msg.data - self.left_enc
        dist = (delta_ticks / self.ticks_per_rev) * (pi * self.wheel_diam)

        self.vl = dist / delta_time
        self.left_vel_pub.publish(self.vl)
        self.left_enc = msg.data

    def right_callback(self, msg):
        if self.right_enc is None:
            self.right_enc = msg.data
            return
            
        now = float(msg.header.stamp.to_nsec())
        delta_time = (now - self.right_last_time) * 1e-9
        self.right_last_time = now

        delta_ticks = msg.data - self.right_enc
        dist = (delta_ticks / self.ticks_per_rev) * (pi * self.wheel_diam)

        self.vr = dist / delta_time
        self.right_vel_pub.publish(self.vr)
        self.right_enc = msg.data

    # use differential drive equations to predict pose. 
    def pose_update(self, delta_t):
        res = None
        omega = (self.vr - self.vl) / self.wheel_space

        if omega == 0:
            self.posx += self.vr * delta_t * cos(self.theta)
            self.posy += self.vr * delta_t * sin(self.theta)
        else:
            radius = (self.wheel_space / 2) * ((self.vr + self.vl) / (self.vr - self.vl))

            cx = self.posx - radius * sin(self.theta)
            cy = self.posy + radius * cos(self.theta)

            self.posx = ((self.posx - cx) * cos(omega * delta_t)) + ((self.posy - cy) * -sin(omega * delta_t)) + cx
            self.posy = ((self.posx - cx) * sin(omega * delta_t)) + ((self.posy - cy) *  cos(omega * delta_t)) + cy
            self.theta += omega * delta_t

        pose_msg = Pose2D()
        pose_msg.x = self.posx
        pose_msg.y = self.posy
        pose_msg.theta = self.theta

        self.pose_pub.publish(pose_msg)

if __name__ == '__main__':
    Localizer()