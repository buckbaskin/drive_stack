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

# tentatively removed, might need it to call services properly
# from drive_stack.srv import Goal

def quaternion_to_heading(quaternion):
    """
    Converts a quaternion to equivalent Euler yaw/heading
    input: nav_msgs.msg.Quaternion
    output: euler heading in radians
    """
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    yaw = tft.euler_from_quaternion(quat)[2]
    return yaw

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

def dot_product(vec1, vec2):
    """
    calcuates the dot product of two tuples/vectors a, b
    input: 2 three-tuples a, vec2
    output: double: dot product
    """
    return vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2]

def cross_product(vec1, vec2):
    """
    calcuates the cross product of two tuples/vectors a, b
    input: 2 three-tuples vec1, vec2
    output: three-tuple (cross product of a, vec2)

     i    j    k
    a[0] a[1] a[2]
    b[0] b[1] b[2]
    """
    i = vec1[1]*vec2[2]-vec1[2]*vec2[1]
    j = vec1[0]*vec2[2]-vec1[2]*vec2[0]
    k = vec1[0]*vec2[1]-vec1[1]*vec2[0]
    return (i, j, k,)

def scale(vector, magnitude):
    """
    scales the given vector by the magnitude (scalar multiplication)
    input: three-tuple vector, double magnitude
    output: three-tuple scaled vector
    """
    return (vector[0]*magnitude, vector[1]*magnitude, vector[2]*magnitude)

def unit(vector):
    """
    returns the unit vector in the same direction as the given vector
    """
    length = math.sqrt(vector[0]*vector[0]+vector[1]*vector[1]+
        vector[2]*vector[2])
    return scale(vector, 1.0/length)

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
        self.position = None
        # ROS services
        self.lead_back = None
        self.lead_start = None
        self.lead_next = None
        self.lead_goal = None

    def wait_for_services(self):
        """
        # OVERRIDE this method to have the node wait for a service or services
        #  before offering its own and beginning publishing. Be careful, because
        #  multiple nodes waiting on each other will maintain blocking calls
        #  indefinitely.

        # Services from Leader that are critical to leader
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
        self.lead_goal = rospy.ServiceProxy('/lead/goal')
        self.lead_next = rospy.ServiceProxy('/lead/next')
        self.lead_start = rospy.ServiceProxy('/lead/start')
        self.lead_back = rospy.ServiceProxy('/lead/back')

    def init_node(self):
        rospy.init_node('default_path')
        self.position = rospy.Subscriber('/odom', Odometry, self.process_odom)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def process_odom(self, odom):
        """
        Takes in a new odom/best position estimate
        sends out a new control command based on this best estimate of location
        Note: this may not work effectively for control. It is an example
        implementation at best.
        """
        next_goal = self.lead_goal()
        if self.dist(next_goal, odom) < .04:
            # if you are .04 m or less from the goal, move forward
            # NOTE: this might change.
            # for example, you may want to look at the next_goal two points
            #  ahead and see if you want to skip the current one, or if you are
            #  ahead of the current one in the direction that you want to go
            next_goal = self.lead_next()

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
        self.cmd_vel.publish(twist_out)

    def calc_errors(self, location, goal):
        along = self.along_axis_error(location, goal)
        off = self.off_axis_error(location, goal)
        heading = self.heading_error(location, goal)
        return (along, off, heading,)

    # pylint: disable=no-self-use
    # these are clearly used. See ^ calc_errors above
    def along_axis_error(self, location, goal):
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

        goal_heading = quaternion_to_heading(goal.pose.pose.position)
        goal_vector_x = math.cos(goal_heading)
        goal_vector_y = math.sin(goal_heading)
        goal_vector_z = 0.0

        # vector in the direction of the goal heading, axis of desired motion
        goal_vector = (goal_vector_x, goal_vector_y, goal_vector_z)

        return dot_product(relative_position, goal_vector)

    def off_axis_error(self, location, goal):
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

        goal_heading = quaternion_to_heading(goal.pose.pose.position)
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
        loc_head = quaternion_to_heading(location.pose.pose.quaternion)
        goal_head = quaternion_to_heading(goal.pose.pose.quaternion)
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
        self.wait_for_services()
        self.init_node()

        rt = rospy.Rate(10)
        while not rospy.is_shutdown():
            rt.sleep()

if __name__ == '__main__':
    # pylint: disable=invalid-name
    # ignoring the trivial naming of the Driver class for startup and run
    d = Driver()
    d.run_node()
