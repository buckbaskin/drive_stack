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
        rospy.spin()

    def process_position(self, odom):
        odom.header.frame_id = 'map'
        self.original_odom.publish(odom)

        print('add noise')

        # noise = np.random.normal(0,1)

        ## noise params ##
        # pose.pose.position.[x,y] 2
        # pose.pose.orientation.[x,y,z,w] (heading) 3
        # twist.twist.linear.[x] 4
        # twist.twist.angular.[z] 5

        odom.pose.pose.position.x += noise(0,1) # 1
        odom.pose.pose.position.y += noise(0,1) # 2

        heading = quaternion_to_heading(odom.pose.pose.orientation)
        heading += noise(0,1)
        odom.pose.pose.orientation = heading_to_quaternion(heading) # 3
        

        self.noisy_odom.publish(odom)

if __name__ == '__main__':
    no = NoisyOdom()