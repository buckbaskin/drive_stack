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
 
 Path

 This is a class that generates high level goals in the map frame
 for a robot to achieve. It is designed to sync with a 
 Leader node that takes the published goals, and then generates
 Odometry targets between them for the robot to follow (think carrot
 on a stick).

 Interface:
 srv/path/goal - returns current goal
 srv/path/next - moves path to next goal, returns that goal
 srv/path/start - returns starting goal
 srv/path/back - moves path to previous goal, returns that goal
 msg/path/current - publishes current goal

 TODO Summary:

 Implement a non-trivial path for example purposes

 ''' #pylint: disable=pointless-string-statement

import rospy
from nav_msgs.msg import Odometry
import drive_stack

class Path(object):
    def __init__(self):
        self.path = []
        self.frame = 'map'
        self.path.append(Odometry(x = 0, y = 1)) # TODO(buckbaskin): change
        self.path.append(Odometry(x = 0, y = 2)) # TODO(buckbaskin): change 
        self.path.append(Odometry(x = 0, y = 3)) # TODO(buckbaskin): change 
        self.index = 0

        self.rolling_index = -1

    # Pub/Sub/Service functionality

    def goal_callback(self):
        return drive_stack.srv.GoalResponse(self.path[self.index+1])

    def next_callback(self):
        if len(self.path) > self.index+2:
            self.index += 1
        return self.goal_callback()

    def start_callback(self):
        return drive_stack.srv.GoalResponse(self.path[self.index])

    def back_callback(self):
        self.index += -1
        if self.index < 0:
            self.index = 0
        return self.goal_callback()

    def current(self):
        return self.goal_callback()

    def start(self):
        return self.start_callback()

    def next_rolling_pub(self):
        self.rolling_index += 1
        self.rolling_index = self.rolling_index % len(self.path)
        return path[rolling_index]

    # Server/running management

    def wait_for_services(self):
        # OVERRIDE this method to have the node wait for a service or services
        #  before offering its own and beginning publishing. Be careful, because
        #  multiple nodes waiting on each other will maintain blocking calls
        #  indefinitely.
        pass

    def init_server(self):
        rospy.init_node('default_path')
        self.goal = rospy.Service('/path/goal', drive_stack.srv.Goal, goal_callback)
        self.next = rospy.Service('/path/next', drive_stack.srv.Goal, next_callback)
        self.start = rospy.Service('/path/start', drive_stack.srv.Goal, start_callback)
        self.back = rospy.Service('/path/back', drive_stack.srv.Goal, back_callback)
        self.current = rospy.Publisher('/path/current', Odometry, queue_size=1)
        self.start_pub = rospy.Publisher('/path/start_goal', Odometry, queue_size=1)
        self.rolling = rospy.Publisher('/path/rolling', Odometry, queue_size=1)

    def publish_path_interface(self):
        self.current.publish(self.current())
        self.start_pub.publish(self.start())
        self.rolling.publish(self.next_rolling_pub())

    def run_server(self):
        self.wait_for_services()
        self.init_server()
        rt = rospy.rate(10)
        while not rospy.is_shutdown():
            self.publish_path_interface()
            rt.sleep()