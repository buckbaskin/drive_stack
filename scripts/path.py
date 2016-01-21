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
'''

import rospy
import math
from nav_msgs.msg import Odometry
from drive_stack.srv import Goal, GoalResponse

from utils import heading_to_quaternion
from utils import easy_Odom

class Path(object):
    """
    Path class: see module doc string

    no constructor arguments

    Example Usage:

    p = Path()
    p.run_server()
    """

    # pylint: disable=too-many-instance-attributes
    # These attributes represent the path, frame, and pub/sub

    def __init__(self, triple=False):

        self.path = []
        if not triple:
            # start
            self.path.append(easy_Odom(x=2, y=2, v=0.5, heading=math.pi/2, frame='map'))
            # out
            self.path.append(easy_Odom(x=2, y=7, v=0.5, heading=math.pi/2, frame='map'))
            # over
            self.path.append(easy_Odom(x=3, y=8, v=0.5, heading=0.0, frame='map'))
            # turned around
            self.path.append(easy_Odom(x=4, y=7, v=0.5, heading=-math.pi/2, frame='map'))
            # back
            self.path.append(easy_Odom(x=4, y=2, v=0.5, heading=-math.pi/2, frame='map'))
            # over
            self.path.append(easy_Odom(x=3, y=1, v=0.5, heading=-math.pi, frame='map'))
            # back to 0rigin
            self.path.append(easy_Odom(x=2, y=2, v=0.0, heading=math.pi/2, frame='map'))
        else:
            # start
            self.path.append(easy_Odom(x=2, y=2, v=0.5, heading=math.pi/2, frame='map'))
            # start cross 1
            self.path.append(easy_Odom(x=3, y=3, v=0.5, heading=0.0, frame='map'))
            # end cross 1
            self.path.append(easy_Odom(x=4, y=3, v=0.5, heading=0.0, frame='map'))
            # up
            self.path.append(easy_Odom(x=5, y=4, v=0.5, heading=math.pi/2, frame='map'))
            # start cross 2
            self.path.append(easy_Odom(x=4, y=5, v=0.5, heading=-math.pi, frame='map'))
            # end cross 2
            self.path.append(easy_Odom(x=2, y=5, v=0.5, heading=-math.pi, frame='map'))
            # up
            self.path.append(easy_Odom(x=1, y=6, v=0.5, heading=math.pi/2, frame='map'))
            # start cross 3
            self.path.append(easy_Odom(x=2, y=7, v=0.0, heading=0.0, frame='map'))
            # end cross 3
            self.path.append(easy_Odom(x=4, y=7, v=0.5, heading=0.0, frame='map'))
            # down
            self.path.append(easy_Odom(x=5, y=5, v=0.5, heading=-math.pi/2, frame='map'))
            # back
            self.path.append(easy_Odom(x=5, y=2, v=0.5, heading=-math.pi/2, frame='map'))
            # over
            self.path.append(easy_Odom(x=3, y=1, v=0.5, heading=-math.pi, frame='map'))
            # back to 0rigin
            self.path.append(easy_Odom(x=2, y=2, v=0.0, heading=math.pi/2, frame='map'))
        self.index = 0

        self.rolling_index = -1

        # ROS publishers, etc. defined on start_server
        self.goal = None
        self.start = None
        self.start_pub = None
        self.back = None
        self.next = None
        self.current = None
        self.rolling = None


    # Pub/Sub/Service functionality

    def goal_callback(self, req=None):
        """
        return the current goal. callback for service
        """
        return GoalResponse(self.path[self.index+1])

    def next_callback(self, req=None):
        """
        return the current goal after stepping forward one. callback for service
        """
        if len(self.path) > self.index+2:
            self.index += 1
        return self.goal_callback()

    def start_callback(self, req=None):
        """
        return the current start point. callback for service
        """
        return GoalResponse(self.path[self.index])

    def back_callback(self, req=None):
        """
        return the start point after stepping back one. callback for service
        """
        self.index += -1
        if self.index < 0:
            self.index = 0
        return self.goal_callback()

    def next_rolling_pub(self):
        """
        return the next item in the path. rolling for RViz visuzalization
        """
        self.rolling_index += 1
        self.rolling_index = self.rolling_index % len(self.path)
        # rospy.loginfo('rlli i: '+str(self.path[self.rolling_index].pose.pose.position.x)+
        #     ','+str(self.path[self.rolling_index].pose.pose.position.y))
        return self.path[self.rolling_index]

    # Server/running management

    def wait_for_services(self):
        """
        wait for necessary services to implement interfaces
        """
        # OVERRIDE this method to have the node wait for a service or services
        #  before offering its own and beginning publishing. Be careful, because
        #  multiple nodes waiting on each other will maintain blocking calls
        #  indefinitely.
        pass

    def init_server(self):
        """
        start the ROS node, including offering services.
        """
        rospy.init_node('default_path')
        # pylint: disable=line-too-long
        # it makes sense for the service initiations to happen on one line
        self.goal = rospy.Service('/path/goal', Goal, self.goal_callback)
        self.next = rospy.Service('/path/next', Goal, self.next_callback)
        self.start = rospy.Service('/path/start', Goal, self.start_callback)
        self.back = rospy.Service('/path/back', Goal, self.back_callback)
        self.current = rospy.Publisher('/path/current', Odometry, queue_size=1)
        self.start_pub = rospy.Publisher('/path/start_goal', Odometry, queue_size=1)
        self.rolling = rospy.Publisher('/path/rolling', Odometry, queue_size=1)

        # for i in range(0, len(self.path)):
        #     rospy.loginfo('path: '+str(i)+' '+str(self.path[i].pose.pose.position.y))


    def publish_path_interface(self):
        """
        publish all path-interface related publishing requirements
        """
        self.current.publish(self.goal_callback("This").goal)
        self.start_pub.publish(self.start_callback().goal)
        # print 'rolling pub'
        self.rolling.publish(self.next_rolling_pub())

    def run_server(self):
        """
        run the node. waits for services, inits server, runs publishers
        """
        self.wait_for_services()
        self.init_server()
        rospy.loginfo('path: server running')
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.publish_path_interface()
            rate.sleep()

if __name__ == '__main__':
    # pylint: disable=invalid-name
    path = Path(True)
    path.run_server()
