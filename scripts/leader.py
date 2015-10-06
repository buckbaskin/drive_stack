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
 
 Leader

 This class takes in high level goals, and then develops a path (a series
 of differential poses) for the robot to follow along. Think carrot
 on a stick, where the robot tries to reach the poses being published.

 Interface:
 srv/lead/target - returns current target
 srv/lead/next - moves path to next , returns that goal
 srv/lead/start - returns starting target
 srv/lead/back - moves path to previous target, returns that target
 msg/lead/current - publishes current target
 '''

import rospy

class Leader(object):
    def __init__(self):
        self.targets = []
        self.frame = 'map'
        self.index = 0
        self.rolling_index = -1

    # Pub/Sub/Service functionality

    def goal_callback(self):
        return drive_stack.srv.GoalResponse(self.targets[self.index+1])

    def next_callback(self):
        if len(self.targets) > self.index+2:
            self.index += 1
            if self.index < 0:
                self.index = 0
            return self.goal_callback()
        else:
            self.generate_next_path(False) # don't reverse
            return self.goal_callback()


    def start_callback(self):
        return drive_stack.srv.GoalResponse(self.targets[self.index])

    def back_callback(self):
        self.index += -1
        if len(self.targets) <= self.index+1:
            self.index = len(self.targets) - 2
            if self.index < 0:
                self.index = 0
            return self.goal_callback()
        else:
            self.generate_next_path(True) # do reverse next path
            return self.goal_callback()

    def current(self):
        return self.goal_callback()

    def start(self):
        return self.start_callback()

    def next_rolling_pub(self):
        self.rolling_index += 1
        self.rolling_index = self.rolling_index % len(self.targets)
        return path[rolling_index]

    # Server/running management

    def wait_for_services(self):
        # OVERRIDE this method to have the node wait for a service or services
        #  before offering its own and beginning publishing. Be careful, because
        #  multiple nodes waiting on each other will maintain blocking calls
        #  indefinitely.

        # Services from Path that are critical to leader
        #  self.goal = rospy.Service('/path/goal', drive_stack.srv.Goal, goal_callback)
        #  self.next = rospy.Service('/path/next', drive_stack.srv.Goal, next_callback)
        #  self.start = rospy.Service('/path/start', drive_stack.srv.Goal, start_callback)
        #  self.back = rospy.Service('/path/back', drive_stack.srv.Goal, back_callback)
        rospy.wait_for_service('/path/goal')
        rospy.wait_for_service('/path/next')
        rospy.wait_for_service('/path/start')
        rospy.wait_for_service('/path/back')

        # Assign callables for the Path services
        self.path_goal = rospy.ServiceProxy('/path/goal')
        self.path_next = rospy.ServiceProxy('/path/next')
        self.path_start = rospy.ServiceProxy('/path/start')
        self.path_back = rospy.ServiceProxy('/path/back')

    def init_server(self):
        rospy.init_node('default_path')

        self.generate_initial_path()

        self.goal = rospy.Service('/lead/goal', drive_stack.srv.Goal, goal_callback)
        self.next = rospy.Service('/lead/next', drive_stack.srv.Goal, next_callback)
        self.start = rospy.Service('/lead/start', drive_stack.srv.Goal, start_callback)
        self.back = rospy.Service('/lead/back', drive_stack.srv.Goal, back_callback)
        self.current = rospy.Publisher('/lead/current', Odometry, queue_size=1)
        self.start_pub = rospy.Publisher('/lead/start_goal', Odometry, queue_size=1)
        self.rolling = rospy.Publisher('/lead/rolling', Odometry, queue_size=1)

    def generate_initial_path(self):
        # TODO(buckbaskin): change to getting path from Path, generating intermediate points
        # Note: this is called once during node initialization
        end = self.path_goal()
        start = self.path_start()

        self.path = []
        self.path.append(Odometry(x = 0, y = 1))
        self.path.append(Odometry(x = 0, y = 2))
        self.path.append(Odometry(x = 0, y = 3))
        self.index = 0

    def generate_next_path(self, rvs):
        # TODO(buckbaskin): change to getting path from Path, generating intermediate points
        # if rvs: restart the current Path segment between goals
        # else: generate a path to the next Path goal
        if not rvs:
            end = self.path_next()
            start = self.path_start()
        else:
            # restart the current segment
            start = self.path_start()
            end = start.path_goal()

        self.path = []
        self.path.append(Odometry(x = 0, y = 1))
        self.path.append(Odometry(x = 0, y = 2))
        self.path.append(Odometry(x = 0, y = 3))
        self.index = 0

    def publish_path_interface(self):
        if len(self.targets)
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