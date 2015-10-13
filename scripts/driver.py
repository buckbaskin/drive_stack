#!/usr/bin/env python

'''
Copyright 2015 William Baskin

/*****************************************
 LICENSE SUMMARY

 This package is licensed under the 
    MIT License. Please see the LICENSE.md
    file in the root folder for the 
    complete license.

 *****************************************/

 Driver

 A Driver is a class that takes in an Odometry
 feed from a Leader class and an Odometry feed
 from its trusted source of localization data
 and then outputs Twist commands for the robot.

 Interface:
 msg/cmd_vel - publishes the desired command/reaction for the robot
 ''' #pylint: disable=pointless-string-statement

import rospy
from geometry_msgs.msg import Odometry, Twist
import drive_stack

class Driver(object):
    def __init__(self):
        pass

    def wait_for_services(self):
        # OVERRIDE this method to have the node wait for a service or services
        #  before offering its own and beginning publishing. Be careful, because
        #  multiple nodes waiting on each other will maintain blocking calls
        #  indefinitely.

        # Services from Leader that are critical to leader
        #  self.goal = rospy.Service('/lead/goal', drive_stack.srv.Goal, goal_callback)
        #  self.next = rospy.Service('/lead/next', drive_stack.srv.Goal, next_callback)
        #  self.start = rospy.Service('/lead/start', drive_stack.srv.Goal, start_callback)
        #  self.back = rospy.Service('/lead/back', drive_stack.srv.Goal, back_callback)
        rospy.wait_for_service('/lead/goal')
        rospy.wait_for_service('/lead/next')
        rospy.wait_for_service('/lead/start')
        rospy.wait_for_service('/lead/back')

        # Assign callables for the Path services
        self.lead_goal = rospy.ServiceProxy('/lead/goal')
        self.lead_next = rospy.ServiceProxy('/lead/next')
        self.lead_start = rospy.ServiceProxy('/lead/start')
        self.lead_back = rospy.ServiceProxy('/lead/back')

    def init_node(self):
        rospy.init_node('default_path')

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def next_cmd(self):
        return new Twist()

    def run_node(self):
        self.wait_for_services()
        self.init_node()
        
        rt = rospy.rate(10)
        while not rospy.is_shutdown():
            self.cmd_vel.publish(next_cmd())
            rt.sleep()

if __name__ == '__main__':
    d = Driver()
    d.run_node()