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
 '''

import rospy
import math
from tf import transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from drive_stack.srv import Goal
from utils import quaternion_to_heading, heading_to_quaternion
from utils import dot_product, cross_product, scale, unit
# tentatively removed, might need it to call services properly
# from drive_stack.srv import Goal

class Driver(object):
    """
    See above
    """
    # pylint: disable=invalid-name
    # constants are somewhat standard, will be changed based on new math
    # see (Kanayama, Fahroo 1997 "A New Line Tracking Method...")
    a = 1
    b = 1
    c = 1
    j = 1
    k = 1

    def __init__(self):
        # ROS pub/sub
        self.cmd_vel = None
        self.silent_cmd = None
        self.position = None
        # ROS services
        self.lead_back = None
        self.lead_start = None
        self.lead_next = None
        self.lead_goal = None

        self.silent = True

    def wait_for_services(self):
        """
        # OVERRIDE this method to have the node wait for a service or services
        #  before offering its own and beginning publishing. Be careful, because
        #  multiple nodes waiting on each other will maintain blocking calls
        #  indefinitely.

        # Services from Leader that are critical to a driver
        #  self.goal = rospy.Service('/lead/goal', Goal, goal_callback)
        #  self.next = rospy.Service('/lead/next', Goal, next_callback)
        #  self.start = rospy.Service('/lead/start', Goal, start_callback)
        #  self.back = rospy.Service('/lead/back', Goal, back_callback)
        """

        rospy.wait_for_service('/lead/goal')
        rospy.wait_for_service('/lead/next')
        rospy.wait_for_service('/lead/start')
        rospy.wait_for_service('/lead/back')

        # Assign callables for the Path services
        self.lead_goal = rospy.ServiceProxy('/lead/goal', Goal)
        self.lead_next = rospy.ServiceProxy('/lead/next', Goal)
        self.lead_start = rospy.ServiceProxy('/lead/start', Goal)
        self.lead_back = rospy.ServiceProxy('/lead/back', Goal)

    def init_node(self):
        """
        Start the pub/sub portion of the ROS node
        """
        rospy.init_node('default_driver')
        self.position = rospy.Subscriber('/base_pose_ground_truth', Odometry, self.process_position)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.silent_cmd = rospy.Publisher('/silent_cmd', Twist, queue_size=1)

    def process_position(self, odom):
        """
        Takes in a new odom/best position estimate
        sends out a new control command based on this best estimate of location
        Note: this may not work effectively for control. It is an example
        implementation at best.
        """
        next_goal = self.lead_goal().goal
        if self.dist(next_goal, odom) < .04:
            # if you are .04 m or less from the goal, move forward
            # NOTE: this might change.
            # for example, you may want to look at the next_goal two points
            #  ahead and see if you want to skip the current one, or if you are
            #  ahead of the current one in the direction that you want to go
            next_goal = self.lead_next().goal

        # errors along axis "x", off axis "y", heading "theta"
        along, off, heading = self.calc_errors(odom, next_goal)

        v_error = odom.twist.twist.linear.x - next_goal.twist.twist.linear.x
        omega_error = (odom.twist.twist.angular.z -
            next_goal.twist.twist.angular.z)

        linear_vel = odom.twist.twist.linear.x
        angular_vel = odom.twist.twist.angular.z

        # adjust speed to account for error in position along axis
        # think damped mass spring system
        # Force = position-correction (spring) + velocity-correction(damper)
        # F(ma) = -kx -b(dx/dt)
        # critically damped (optimal) is b^2 = 4mk
        # ma + bv + kx = 0
        # accel desired (correction) = -bv - kx

        # for now, just a linear weighted correction
        linear_vel += -self.k*along
        linear_vel += -self.b*v_error

        # angular velocity needs to adjust for heading error
        # as well as offset from the desired axis

        # F = -j(theta) - a(dtheta/dt)
        angular_vel += -self.j*heading
        angular_vel += -self.a*omega_error

        # correction for offset from axis
        # positive error is to the left, so correction is to the right/negative
        # linear-like assumption
        # correction = -c*off , where off is the error off axis
        angular_vel += -self.c*off

        twist_out = Twist()
        twist_out.linear.x = linear_vel
        twist_out.angular.z = angular_vel
        if self.silent or True:
            self.silent_cmd.publish(twist_out)
        else:
            self.cmd_vel.publish(twist_out)

    def calc_errors(self, location, goal):
        """
        calculate errors in "x", "y", "theta" between a location and a goal

        input: two nav_msgs.msg.Odometry, a current best estimate of location
         and the goal
        output: a three-tuple representing error along the goal heading vector,
         error normal to that vector, and heading error

        example usage:
        odom = Odometry(current location)
        goal = Odometry(target location)
        along_axis, off_axis, heading = self.calc_errors(odom, goal)
        """
        along = self.along_axis_error(location, goal)
        off = self.off_axis_error(location, goal)
        heading = self.heading_error(location, goal)
        rospy.loginfo('a: %d o: %d h: %d' % (along, off, heading,))
        return (along, off, heading,)

    # pylint: disable=no-self-use
    # these are clearly used. See ^ calc_errors above
    def along_axis_error(self, location, goal):
        """
        calc error along the axis defined by the goal position and direction

        input: two nav_msgs.msg.Odometry, current best location estimate + goal
        output: double distance along the axis

        axis is defined by a vector from the unit circle aligned with the goal
         heading
        relative position is the vector from the goal x, y to the location x, y

        distance is defined by the dot product

        example use:
        see calc_errors above
        """
        relative_position_x = (location.pose.pose.position.x -
            goal.pose.pose.position.x)
        relative_position_y = (location.pose.pose.position.y -
            goal.pose.pose.position.y)

        # relative position of the best estimate position and the goal
        # vector points from the goal to the location
        relative_position = (relative_position_x, relative_position_y, 0.0)

        goal_heading = quaternion_to_heading(goal.pose.pose.orientation)
        goal_vector_x = math.cos(goal_heading)
        goal_vector_y = math.sin(goal_heading)

        # vector in the direction of the goal heading, axis of desired motion
        goal_vector = (goal_vector_x, goal_vector_y, 0.0)

        return dot_product(relative_position, goal_vector)

    def off_axis_error(self, location, goal):
        """
        calc error normal to axis defined by the goal position and direction

        input: two nav_msgs.msg.Odometry, current best location estimate and
         goal
        output: double distance along the axis

        axis is defined by a vector from the unit circle aligned with the goal
         heading
        relative position is the vector from the goal x, y to the location x, y

        distance is defined by subtracting the parallel vector from the total
         relative position vector

        example use:
        see calc_errors above
        """
        relative_position_x = (location.pose.pose.position.x -
            goal.pose.pose.position.x)
        relative_position_y = (location.pose.pose.position.y -
            goal.pose.pose.position.y)
        relative_position_z = (location.pose.pose.position.z -
            goal.pose.pose.position.z)

        # relative position of the best estimate position and the goal
        # vector points from the goal to the location
        relative_position = (relative_position_x, relative_position_y,
            relative_position_z)

        goal_heading = quaternion_to_heading(goal.pose.pose.orientation)
        goal_vector_x = math.cos(goal_heading)
        goal_vector_y = math.sin(goal_heading)

        # vector in the direction of the goal heading, axis of desired motion
        goal_vector = (goal_vector_x, goal_vector_y, 0.0)

        relative_along_goal = scale(unit(relative_position),
            dot_product(relative_position, goal_vector))

        relative_normal_x = relative_position[0]-relative_along_goal[0]
        relative_normal_y = relative_position[1]-relative_along_goal[1]
        relative_normal_z = relative_position[2]-relative_along_goal[2]

        return math.sqrt(relative_normal_x*relative_normal_x+
            relative_normal_y*relative_normal_y+
            relative_normal_z*relative_normal_z)


    def heading_error(self, location, goal):
        """
        return difference in heading between location and goal
        """
        loc_head = quaternion_to_heading(location.pose.pose.orientation)
        goal_head = quaternion_to_heading(goal.pose.pose.orientation)
        return loc_head - goal_head

    def dist(self, odom1, odom2):
        """
        returns linear distance between two odometry messages
        """
        # pylint: disable=invalid-name
        # x and y accurately represent the axis that I'm referring to
        x = odom1.pose.pose.position.x - odom2.pose.pose.position.x
        y = odom1.pose.pose.position.y - odom2.pose.pose.position.y
        return math.sqrt(x*x+y*y)

    def run_node(self):
        """
        Runs the ROS node (initialization, cyclical pub/sub)
        """
        self.wait_for_services()
        self.init_node()
        rospy.loginfo('driver: node ready')
        rt = rospy.Rate(10)
        rospy.spin()

if __name__ == '__main__':
    # pylint: disable=invalid-name
    # ignoring the trivial naming of the Driver class for startup and run
    d = Driver()
    d.run_node()
