#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

import numpy as np
from numpy.random import normal as noise
from utils import quaternion_to_heading, heading_to_quaternion

class NoisyOdom(object):
    def __init__(self):
        rospy.init_node('noisy_odom')
        self.odom = rospy.Subscriber('/base_pose_ground_truth', Odometry, self.process_position)
        self.noisy_odom = rospy.Publisher('/ekf_simulated', Odometry, queue_size=1)
        self.original_odom = rospy.Publisher('/ekf_clean', Odometry, queue_size=1)

        self.position_variation = .01 # +- .1 m
        self.heading_variation = .0175 # approx 10 degrees
        self.linear_vel = .01 # +- .1 m/s
        self.angular_vel = .01 # +- .1 rad/sec

        rospy.loginfo('noisy odom running.')
        rospy.spin()

    def process_position(self, odom):
        odom.header.frame_id = 'map'
        self.original_odom.publish(odom)

        # print('add noise')

        # noise = np.random.normal(0,1)

        ## noise params ##
        # pose.pose.position.[x,y] 2
        # pose.pose.orientation.[x,y,z,w] (heading) 3
        # twist.twist.linear.[x] 4
        # twist.twist.angular.[z] 5

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        v = abs(odom.twist.twist.linear.x)
        w = abs(odom.twist.twist.angular.z)

        odom.pose.pose.position.x += noise(0,self.position_variation) # 1
        odom.pose.pose.position.y += noise(0,self.position_variation) # 2

        if odom.pose.pose.position.z: # if it is not 0
            odom.pose.pose.position.z +=  noise(0,self.position_variation)

        heading = quaternion_to_heading(odom.pose.pose.orientation)
        heading += noise(0,self.heading_variation+.05*w+.01*v)
        odom.pose.pose.orientation = heading_to_quaternion(heading) # 3

        odom.twist.twist.linear.x += noise(0,self.linear_vel+.05*v+.01*w) # 4

        if odom.twist.twist.linear.y: # if it is not 0
            odom.twist.twist.linear.y +=  noise(0,self.linear_vel)
        if odom.twist.twist.linear.z: # if it is not 0
            odom.twist.twist.linear.z +=  noise(0,self.linear_vel)

        odom.twist.twist.angular.z += noise(0,self.angular_vel+.05*w+.01*v) # 5

        if odom.twist.twist.angular.x: # if it is not 0
            odom.twist.twist.angular.x +=  noise(0,self.angular_vel)
        if odom.twist.twist.angular.y: # if it is not 0
            odom.twist.twist.angular.y +=  noise(0,self.angular_vel)

        self.noisy_odom.publish(odom)


if __name__ == '__main__':
    no = NoisyOdom()