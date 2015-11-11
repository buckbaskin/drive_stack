#!/usr/bin/env python

import leader
import rospy
import math
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry

from leader import heading_to_quaternion

def to_unit_tuple(v_tuple):
    dist = 0
    dist += pow(v_tuple[0],2)
    dist += pow(v_tuple[1],2)
    dist = sqrt(dist)

    return (v_tuple[0]/dist, v_tuple[1]/dist,)

def unit(function):
    def modded_function(vec_as_tuple):
        val = function(vec_as_tuple)
        return to_unit_tuple(val)
    return modded_function

class ForceLeader(leader.Leader):
    # methods to override:
    # generate_initial_path, generate_next_path
    # this is the same implementation as the Leader class, but separate to 
    # demonstrate how to override it.

    def generate_initial_path(self):
        """
        Path creation for node
        """
        # TODO(buckbaskin): Completely revise this
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
        # TODO(buckbaskin): Completely revise this
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

    def get_force_vector(self, start, end, x, y):
        """
        Aggregate the sum of the force vectors for 3 different actors: departing
        the original location, arriving at the destination and traversing 
        between the two points. 

        There may be more added, and the end output is 
        only relevant for its direction, so the weighting is arbitrary and 
        relative, where I've fixed the traverse weight to always be 1. 

        This is a superposition of 3 differential equations representing a sum 
        of forces.
        """
        wdep = self.weighted_depart(start, end, x, y)
        warr = self.weighted_arrive(start, end, x, y)
        wtrv = self.weighted_traverse(start, end, x, y)
        # wobs = self.weighted_obstacle(start, end, x, y)

        force = (wdep[0]+warr[0]+wtrv[0], wdep[1]+warr[1]+wtrv[1],)

        return force

    def weighted_depart(self, start, end, x, y):
        dv = self.depart_vector(self, start, end, x, y)
        w = self.depart_weight(self, start, end, x, y)
        return (dv[0]*w,dv[1]*w,)

    @unit # forces a unit vector to be returned, scaled by weight
    def depart_vector(self, start, end, x, y):
        # TODO(buckbaskin): define to be a "line-source" for departing start
        return (1.0,0.0,)

    def depart_weight(self, start, end, x, y):
        # TODO(buckbaskin): define to scale down with distance
        return 1.0

    def weighted_arrive(self, start, end, x, y):
        av = self.arrive_vector(self, start, end, x, y)
        w = self.arrive_weight(self, start, end, x, y)
        return (av[0]*w,av[1]*w,)

    @unit # forces a unit vector to be returned, scaled by weight
    def arrive_vector(self, start, end, x, y):
        # TODO(buckbaskin): define to be a "line-source" for departing start
        return (1.0,0.0,)

    def arrive_weight(self, start, end, x, y):
        # TODO(buckbaskin): define to scale down with distance
        return 1.0

    def weighted_traverse(self, start, end, x, y):
        tv = self.traverse_vector(self, start, end, x, y)
        w = self.traverse_weight(self, start, end, x, y)
        return (tv[0]*w,tv[1]*w,)

    @unit # forces a unit vector to be returned, scaled by weight
    def traverse_vector(self, start, end, x, y):
        dx = end.pose.pose.position.x - start.pose.pose.position.x
        dy = end.pose.pose.position.y - start.pose.pose.position.y
        return (dx,dy,)

    def traverse_weight(self, start, end, x, y):
        # This is the standard unit. All other forces can use a weight of 1 to 
        #  be of equal weight to the traverse weight. More indicates a greater
        #  requested priority.
        return 1.0


if __name__ == '__main__':
    # pylint: disable=invalid-name
    # leader is a fine name, it's not a constant
    leader = ForceLeader()
    leader.run_server()
