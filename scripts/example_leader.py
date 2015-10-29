#!/usr/bin/env python

# import sys
# print sys.path

import leader
import rospy
import math
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry

from leader import heading_to_quaternion

class ExampleLeader(leader.Leader):
    # methods to override:
    # generate_initial_path, generate_next_path
    # this is the same implementation as the Leader class, but separate to 
    # demonstrate how to override it.

    def generate_initial_path(self):
        """
        Path creation for node
        """
        # Note: this is called once during node initialization
        end = self.path_goal().goal # Odometry
        start = self.path_start().goal # Odometry
        start.header.frame_id = 'odom'
        self.targets = []
        self.targets.append(start)

        # pylint: disable=invalid-name
        # dt, dx, dy properly express what I'm trying to get across
        # i.e. differential time, x, y

        dt = .1
        des_speed = .5 # m/s
        dx = end.pose.pose.position.x - start.pose.pose.position.x
        dy = end.pose.pose.position.y - start.pose.pose.position.y
        # total dx above
        heading = math.atan2(dy, dx)
        step_x = des_speed*math.cos(heading)*dt
        step_y = des_speed*math.sin(heading)*dt
        rospy.loginfo('step_x: '+str(step_x))
        distance = math.sqrt(dx*dx+dy*dy)
        steps = math.floor(distance/(des_speed*dt))

        for i in range(1, int(steps)+1):
            odo = Odometry()
            odo.header.frame_id = 'odom'
            odo.pose.pose.position = Point(x=start.pose.pose.position.x+i*step_x, y=start.pose.pose.position.y+i*step_y)
            rospy.loginfo('gen x: '+str(start.pose.pose.position.x+i*step_x))
            rospy.loginfo('gen y: '+str(start.pose.pose.position.y+i*step_y))
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
            end = self.path_next().goal
            start = self.path_start().goal
        else:
            # move back one segment
            start = self.path_back().goal
            end = start.path_goal().goal

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
            odo.header.frame_id = 'odom'
            odo.pose.pose.point = Point(x=start.x+i*dx, y=start.y+i*dy)
            odo.pose.pose.orientation = heading_to_quaternion(heading)
            odo.twist.twist.linear = Vector3(x=des_speed)
            odo.twist.twist.angular = Vector3()
            self.targets.append(odo)

        if rvs:
            self.index = len(self.targets)-2
        else:
            self.index = 0

if __name__ == '__main__':
    # pylint: disable=invalid-name
    # leader is a fine name, it's not a constant
    leader = ExampleLeader()
    leader.run_server()
