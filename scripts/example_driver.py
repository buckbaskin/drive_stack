#!/usr/bin/env python
"""
Example Driver
Example implementation of the driver

process_position is the key method here
"""

import driver
import rospy

import math
import sys
# from driver import heading_to_quaternion, quaternion_to_heading
# from driver import dot_product, cross_product, scale, unit
# import rospy
from geometry_msgs.msg import Twist

class ExampleDriver(driver.Driver):
    """
    See module docstring
    """
    smoothness = 10
    k = 1.0/smoothness
    a = 3*k
    b = 3*pow(k, 2)
    c = pow(k, 3)

    max_v = 1.0
    max_accel = .05
    max_omega = 0.5
    max_alpha = .05

    # pylint: disable=no-self-use
    # incorrectly identifies helpers as no-self-use

    def __init__(self):
        super(ExampleDriver, self).__init__()
        self.last_odom = None

    def process_position(self, odom):
        """
        Converts new best estimate of position to command
        Based on theory from:
        Y.J. Kanayama and F. Fahroo
        "A New Line Tracking Method for Non-Holonomic Vehicles,"
        in International Conference on Robotics and Automation,
        Albuquerque, NM, 1997, pp.2908-2913
        """
        if self.last_odom is None:
            self.last_odom = odom

        next_goal = self.lead_goal().goal
        while self.advance_next_goal(odom, next_goal):
            # if you are .04 m or less from the goal, move forward
            # NOTE: this might change.
            # for example, you may want to look at the next_goal two points
            #  ahead and see if you want to skip the current one, or if you are
            #  ahead of the current one in the direction that you want to go
            next_goal = self.lead_next().goal

        # errors along axis "x", off axis "y", heading "theta"
        along, off, heading = self.calc_errors(odom, next_goal)

        # d(kurvature)/ds = -a(kurvature) - b(heading_error) - c(off_error)
        # kurvature is defined as: 1/instant radius
        # v = rw
        # w = v/r
        # instant radius = v/w

        # choose a, b, c based on:
        # a = 3k
        # b = 3(k^2)
        # c = k^3
        # choose k based on smoothness = 1/k

        # implement the math
        last_radius = self.calc_old_radius(odom.twist.twist.linear.x,
            odom.twist.twist.angular.z)

        kurvature = self.kurvature_from_radius(last_radius)

        dtime = (odom.header.stamp.nsecs - self.last_odom.header.stamp.nsecs)*pow(10,-9)
        ds = dtime*odom.twist.twist.linear.x
        delta_kurv_discrete = (-1.0*self.a*kurvature -
            self.b*heading - self.c*off)*ds

        new_kurvature = kurvature+delta_kurv_discrete
        new_radius = self.radius_from_kurvature(new_kurvature)
        
        # match speed to goal, with adjustment for position error
        linear_vel = next_goal.twist.twist.linear.x
        linear_vel += -0.5*along

        linear_vel = self.check_linear_limits(odom, linear_vel)
        # calculate angular velocity
        if new_radius > 10000:
            angular_vel = 0
        else:
            angular_vel = linear_vel / new_radius

        angular_vel = self.check_angular_limits(odom, angular_vel)
        # reset linear_vel based on new (lower, limited) angular vel
        if new_radius > 10000 or angular_vel < .0001:
            pass
        else:
            linear_vel = angular_vel * new_radius
        
        # linear and angular velocity are now within dx/dt, d2x/dt2 limits
        twist_out = Twist()
        twist_out.linear.x = linear_vel
        twist_out.angular.z = angular_vel
        rospy.loginfo('lin: '+str(linear_vel)+' ang: '+str(angular_vel))

        if math.isnan(linear_vel) or math.isnan(angular_vel):
            linear_vel = 0
            angular_vel = 0
            twist_out.linear.x = linear_vel
            twist_out.angular.z = angular_vel
            self.cmd_vel.publish(twist_out)
            sys.exit(0)
        self.cmd_vel.publish(twist_out)

    def calc_old_radius(self, linear_vel, angular_vel):
        """
        Calculate an instant radius from a linear velocity and angular velocity
        """
        if angular_vel < .001:
            last_radius = float("inf")
        else:
            last_radius = abs(linear_vel) / abs(angular_vel)
        return last_radius

    def kurvature_from_radius(self, radius):
        """
        Calculate the instant kurvature from an instant radius
        """
        if radius < .001:
            kurvature = 1000 # upper bound on kurvature
        else:
            kurvature = 1.0 / radius
        return kurvature

    def radius_from_kurvature(self, k):
        """
        Calculate the instant radius from the kurvature
        """
        rospy.loginfo('radius_from_kurvature')
        if k < .001:
            new_radius = float("inf")
        else:
            new_radius = 1.0/k
        return new_radius

    def check_linear_limits(self, odom, linear_vel):
        """
        Make sure the given linear velocity fits within accel/speed limits
        """
        # limit maximum acceleration
        if odom.twist.twist.linear.x + self.max_accel < linear_vel:
            linear_vel = odom.twist.twist.linear.x + self.max_accel
        elif linear_vel < odom.twist.twist.linear.x - self.max_accel:
            linear_vel = odom.twist.twist.linear.x - self.max_accel

        # limit maximum speed
        if linear_vel > self.max_v:
            linear_vel = self.max_v
        elif linear_vel < -1.0*self.max_v:
            linear_vel = -1.0*self.max_v

        return linear_vel

    def check_angular_limits(self, odom, angular_vel):
        """
        Make sure the given angular velocity fits within accel/speed limits
        """
        # limit maximum angular acceleration
        if odom.twist.twist.angular.z + self.max_alpha < angular_vel:
            angular_vel = odom.twist.twist.angular.z + self.max_alpha
        elif angular_vel < odom.twist.twist.angular.z - self.max_alpha:
            angular_vel = odom.twist.twist.angular.z - self.max_alpha

        # limit maximum angular speed
        if angular_vel > self.max_omega:
            angular_vel = self.max_omega
        elif angular_vel < -1.0*self.max_omega:
            angular_vel = -1.0*self.max_omega

        return angular_vel

    def advance_next_goal(self, odom, current):
        along, off, heading = self.calc_errors(odom, current)
        return along >= 0.0

if __name__ == '__main__':
    # pylint: disable=invalid-name
    # ignoring the trivial naming of the ExampleDriver class for startup and run
    d = ExampleDriver()
    d.run_node()
