#!/usr/bin/env python

'''
run the following test with:
python driver_mathTest.py
'''

import unittest
import math

import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from driver_pseudolinear import PseudoLinearDriver as Driver
from utils import heading_to_quaternion, quaternion_to_heading, easy_Odom, is_close

class TestDriverCalculations(unittest.TestCase):

    # sign conventions (revised):
    # axis: x axis is parallel to goal, y axis is to- the left when facing
    #  the goal direction, z-axis is oriented up
    # positive heading error - rotated counter clockwise from goal
    # positve offset error - positive y-axis, 

    def setUp(self):
        self.driver_obj = Driver()

    def test_zero(self):
        heading = 0
        offset = 0
        adjusted_heading = self.driver_obj.calc_adjusted_heading(heading, offset)
        self.assertTrue(is_close(adjusted_heading, 0.0))

    def test_pos_heading(self):
        heading = 1.0
        offset = 0
        adjusted_heading = self.driver_obj.calc_adjusted_heading(heading, offset)
        self.assertTrue(is_close(adjusted_heading, heading))

    def test_neg_heading(self):
        heading = -1.0
        offset = 0
        adjusted_heading = self.driver_obj.calc_adjusted_heading(heading, offset)
        self.assertTrue(is_close(adjusted_heading, heading))

    def test_pure_offset1(self):
        heading = 0
        offset = .5
        adjusted_heading = self.driver_obj.calc_adjusted_heading(heading, offset)
        self.assertTrue(adjusted_heading > 0.0)
        self.assertTrue(is_close(adjusted_heading, .75*math.pi/2, 4))

    def test_pure_offset2(self):
        heading = 0
        offset = -.5
        adjusted_heading = self.driver_obj.calc_adjusted_heading(heading, offset)
        self.assertTrue(adjusted_heading < 0.0)
        self.assertTrue(is_close(adjusted_heading, -.75*math.pi/2, 4))

    def test_angular_vel1(self):
        adjusted_heading = 0
        ang_vel = self.driver_obj.calc_angular_velocity(adjusted_heading)
        self.assertTrue(is_close(ang_vel, 0.0))

    def test_angular_vel2(self):
        adjusted_heading = 0.5
        ang_vel = self.driver_obj.calc_angular_velocity(adjusted_heading)
        self.assertTrue(ang_vel < 0.0)
        self.assertTrue(is_close(ang_vel, -0.5))

    def test_angular_vel3(self):
        adjusted_heading = -0.5
        ang_vel = self.driver_obj.calc_angular_velocity(adjusted_heading)
        self.assertTrue(ang_vel > 0.0)
        self.assertTrue(is_close(ang_vel, 0.5))

    def test_angular_vel4(self):
        adjusted_heading = 0.25
        ang_vel = self.driver_obj.calc_angular_velocity(adjusted_heading)
        self.assertTrue(ang_vel < 0.0)
        self.assertTrue(is_close(ang_vel, -0.4333), 3)

    def test_angular_vel5(self):
        adjusted_heading = -0.25
        ang_vel = self.driver_obj.calc_angular_velocity(adjusted_heading)
        self.assertTrue(ang_vel > 0.0)
        self.assertTrue(is_close(ang_vel, 0.4333), 3)

    def test_angular_vel6(self):
        adjusted_heading = 100.0
        ang_vel = self.driver_obj.calc_angular_velocity(adjusted_heading)
        self.assertTrue(ang_vel < 0.0)
        self.assertTrue(is_close(ang_vel, -self.driver_obj.max_omega))

    def test_angular_vel7(self):
        adjusted_heading = -100.0
        ang_vel = self.driver_obj.calc_angular_velocity(adjusted_heading)
        self.assertTrue(ang_vel > 0.0)
        self.assertTrue(is_close(ang_vel, self.driver_obj.max_omega))

    def test_linear_veloicty1(self):
        along = 0.0
        off = 0.0
        ang_vel = 0.0
        goal_vel = 0.0
        lin_vel = self.driver_obj.calc_linear_velocity(along, off, ang_vel, goal_vel)
        self.assertTrue(is_close(lin_vel, 0.0))

    def test_linear_veloicty2(self):
        along = 0.0
        off = 0.0
        ang_vel = 0.0
        goal_vel = self.driver_obj.max_v
        lin_vel = self.driver_obj.calc_linear_velocity(along, off, ang_vel, goal_vel)
        self.assertTrue(is_close(lin_vel, self.driver_obj.max_v))

    def test_linear_veloicty3(self):
        along = 0.0
        off = 0.0
        ang_vel = 0.0
        goal_vel = -self.driver_obj.max_v
        lin_vel = self.driver_obj.calc_linear_velocity(along, off, ang_vel, goal_vel)
        self.assertTrue(is_close(lin_vel, -self.driver_obj.max_v))

    def test_linear_veloicty4(self):
        along = 0.5
        off = 0.0
        ang_vel = 0.0
        goal_vel = 0.0
        lin_vel = self.driver_obj.calc_linear_velocity(along, off, ang_vel, goal_vel)
        self.assertTrue(is_close(lin_vel, -0.5))

    def test_linear_veloicty5(self):
        along = -0.5
        off = 0.0
        ang_vel = 0.0
        goal_vel = 0.0
        lin_vel = self.driver_obj.calc_linear_velocity(along, off, ang_vel, goal_vel)
        self.assertTrue(is_close(lin_vel, 0.5))


if __name__ == '__main__':
    unittest.main()