#!/usr/bin/env python

'''
Copyright 2016 William Baskin

/*****************************************
 LICENSE SUMMARY

 This package is licensed under the
    MIT License. Please see the LICENSE.md
    file in the root folder for the
    complete license.

 *****************************************/

 Odom Trace

 Records all published odom/ground truth poses and performs a rolling publish of
 the recieved positions to /replay.

 Interface:
 msg/replay - publishes the past Odometry positions
 '''

import rospy
from nav_msgs.msg import Odometry

class OdomTrace(object):
    def __init__(self):
        rospy.init_node('odom_trace')
        rospy.loginfo('Trace Running.')
        self.rcvd = []
        self.index = 0
        self.odom = rospy.Subscriber('/odom', Odometry, self.process_position)
        self.replay = rospy.Publisher('/replay', Odometry, queue_size=1)
        sleep_rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            for position in self.rcvd:
                self.replay.publish(position)
                sleep_rate.sleep()
            sleep_rate.sleep()

    def process_position(self, msg):
        self.rcvd.append(msg)

if __name__ == '__main__':
    ot = OdomTrace()
