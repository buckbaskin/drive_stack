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
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3, Quaternion
from drive_stack.srv import Goal, GoalResponse

from rostype import returns
from tf import transformations as tft

def heading_to_quaternion(heading):
    """
    Converts a Euler yaw/heading angle to equivalent quaternion
    input: euler heading in radians
    output: nav_msgs.msg.Quaternion
    """

    quat = tft.quaternion_from_euler(0, 0, heading)

    quaternion = Quaternion()
    quaternion.x = quat[0]
    quaternion.y = quat[1]
    quaternion.z = quat[2]
    quaternion.w = quat[3]
    return quaternion

class Leader(object):
    """
    See above
    """
    # pylint: disable=too-many-instance-attributes
    # there are only 4 non-pub/sub related attributes

    def __init__(self):

        self.targets = []
        self.frame = 'map'
        self.index = 0
        self.rolling_index = -1

        self.path_goal = None
        self.goal = None
        self.path_start = None
        self.start_pub = None
        self.path_back = None
        self.back = None
        self.next = None
        self.current = None
        self.start = None
        self.rolling = None
        self.path_next = None

    # Pub/Sub/Service functionality

    def goal_callback(self, req=None):
        """
        return the current goal. callback for service
        """
        if not(len(self.targets) > self.index+2):
            self.index = len(self.targets) - 2
        return GoalResponse(self.targets[self.index+1])

    def next_callback(self, req=None):
        """
        return the current goal after advancing. callback for service
        """
        if len(self.targets) > self.index+2:
            self.index += 1
            if self.index < 0:
                self.index = 0
            return self.goal_callback()
        else:
            self.generate_next_path(False) # don't reverse
            return self.goal_callback()


    def start_callback(self, req=None):
        """
        return the current starting point. callback for service
        """
        return GoalResponse(self.targets[self.index])

    def back_callback(self, req=None):
        """
        return the starting point after dropping back one. callback for service
        """
        self.index += -1
        if len(self.targets) <= self.index+1:
            self.index = len(self.targets) - 2
            if self.index < 0:
                self.index = 0
            return self.goal_callback()
        else:
            self.generate_next_path(True) # do reverse next path
            return self.goal_callback()

    def next_rolling_pub(self):
        """
        return the rolling publisher for Rviz
        """
        self.rolling_index += 1
        self.rolling_index = self.rolling_index % len(self.targets)
        return self.targets[self.rolling_index]

    # Server/running management

    def wait_for_services(self):
        """
        # OVERRIDE this method to have the node wait for a service or services
        #  before offering its own and beginning publishing. Be careful, because
        #  multiple nodes waiting on each other will maintain blocking calls
        #  indefinitely.

        # pylint: disable=line-too-long
        # services are okay to define on one line that is too long

        # Services from Path that are critical to a leader
        #  self.goal = rospy.Service('/path/goal', Goal, goal_callback)
        #  self.next = rospy.Service('/path/next', Goal, next_callback)
        #  self.start = rospy.Service('/path/start', Goal, start_callback)
        #  self.back = rospy.Service('/path/back', Goal, back_callback)
        """

        rospy.loginfo('waiting for services')
        
        rospy.wait_for_service('/path/goal')
        
        rospy.loginfo('/path/goal works')
        
        rospy.wait_for_service('/path/next')
        
        rospy.loginfo('/path/next works')
        
        rospy.wait_for_service('/path/start')
        
        rospy.loginfo('/path/start works')
        
        rospy.wait_for_service('/path/back')
        
        rospy.loginfo('All services available (x4)')

        # Assign callables for the Path services
        # returns(Odometry)(func) is a decorator that I wrote that forces the
        #  rospy.ServiceProxy('channel') to return the proper type or throw
        #  an error.
        self.path_goal = (rospy.ServiceProxy('/path/goal', Goal))
        self.path_next = (rospy.ServiceProxy('/path/next', Goal))
        self.path_start = (rospy.ServiceProxy('/path/start', Goal))
        self.path_back = (rospy.ServiceProxy('/path/back', Goal))

    def init_server(self):
        """
        Run the ROS node
        """
        rospy.init_node('default_leader')
        self.wait_for_services()
        self.generate_initial_path()
        
        # pylint: disable=line-too-long
        # services are okay to define on one line that is too long

        self.goal = rospy.Service('/lead/goal', Goal, self.goal_callback)
        self.next = rospy.Service('/lead/next', Goal, self.next_callback)
        self.start = rospy.Service('/lead/start', Goal, self.start_callback)
        self.back = rospy.Service('/lead/back', Goal, self.back_callback)
        self.current = rospy.Publisher('/lead/current', Odometry, queue_size=1)
        self.start_pub = rospy.Publisher('/lead/start_goal', Odometry, queue_size=1)
        self.rolling = rospy.Publisher('/lead/rolling', Odometry, queue_size=1)
        rospy.loginfo('leader: server intialized')

    def generate_initial_path(self):
        """
        Path creation for node
        """
        rospy.loginfo('generate_initial_path!!!')
        # Note: this is called once during node initialization
        end = self.path_goal().goal # Odometry
        start = self.path_start().goal # Odometry

        self.targets = []
        self.targets.append(start)

        # pylint: disable=invalid-name
        # dt, dx, dy properly express what I'm trying to get across
        # i.e. differential time, x, y

        dt = .1
        des_speed = .5 # m/s
        dx = end.pose.pose.position.x - start.pose.pose.position.x
        dy = end.pose.pose.position.y - start.pose.pose.position.y

        heading = math.atan2(dy, dx)
        dx = des_speed*math.cos(heading)*dt
        dy = des_speed*math.sin(heading)*dt

        distance = math.sqrt(dx*dx+dy*dy)
        steps = math.floor(distance/des_speed)
        
        for i in range(1, int(steps)):
            odo = Odometry()
            odo.pose.pose.point = Point(x=start.x+i*dx, y=start.y+i*dy)
            odo.pose.pose.orientation = heading_to_quaternion(heading)
            odo.twist.twist.linear = Vector3(x=des_speed)
            odo.twist.twist.angular = Vector3()
            self.targets.append(odo)

        self.index = 0

    def generate_next_path(self, rvs):
        """
        generate a new path, either forwards or backwards (rvs == True)
        """
        # if rvs: move to the previous segement on the path, starting at the end
        # else: generate a path to the next Path goal
        if not rvs:
            end = self.path_next()
            start = self.path_start()
        else:
            # move back one segment
            start = self.path_back()
            end = start.path_goal()

        self.targets = []
        self.targets.append(start)

        # pylint: disable=invalid-name
        # dt, dx, dy properly express what I'm trying to get across
        # i.e. differential time, x, y

        dt = .1
        des_speed = .5 # m/s
        dx = end.x - start.x
        dy = end.y - start.y

        heading = math.atan2(dy, dx)
        dx = des_speed*math.cos(heading)*dt
        dy = des_speed*math.sin(heading)*dt

        distance = math.sqrt(dx*dx+dy*dy)
        steps = math.floor(distance/des_speed)

        for i in range(1, steps):
            odo = Odometry()
            odo.pose.pose.point = Point(x=start.x+i*dx, y=start.y+i*dy)
            odo.pose.pose.orientation = heading_to_quaternion(heading)
            odo.twist.twist.linear = Vector3(x=des_speed)
            odo.twist.twist.angular = Vector3()
            self.targets.append(odo)

        if rvs:
            self.index = len(self.targets)-2
        else:
            self.index = 0
        rospy.loginfo('leader: new path created')

    def publish_leader_interface(self):
        """
        publish on all of the path interface topics
        """
        if len(self.targets) >= 2:
            self.current.publish(self.goal_callback().goal)
            self.start_pub.publish(self.start_callback().goal)
            self.rolling.publish(self.next_rolling_pub())

    def run_server(self):
        """
        Run the node
        """
        self.init_server()
        rospy.loginfo('leader: server running')
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.publish_leader_interface()
            rate.sleep()

if __name__ == '__main__':
    # pylint: disable=invalid-name
    # leader is a fine name, it's not a constant
    leader = Leader()
    leader.run_server()
