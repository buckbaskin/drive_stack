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

from driver import Driver
from utils import heading_to_quaternion, quaternion_to_heading, easy_Odom, is_close

class TestDriverCalculations(unittest.TestCase):

    # sign conventions (revised):
    # axis: x axis is parallel to goal, y axis is to- the left when facing
    #  the goal direction, z-axis is oriented up
    # positive heading error - rotated counter clockwise from goal
    # positve offset error - positive y-axis, 

    def setUp(self):
        self.driver_obj = Driver()

    def test_equal_odom(self):
        # print('test_equal_odom')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, 0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_aheadx_target1(self):
        # print('test_ahead_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=1.0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, -1.0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_aheadx_target2(self):
        # print('test_ahead_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=2.0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, -2.0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_aheadx_target3(self):
        # print('test_ahead_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=3.0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, -3.0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_behindx_target1(self):
        # print('test_behind_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=-1.0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, 1.0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_behindx_target2(self):
        # print('test_behind_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=-2.0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, 2.0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_behindx_target3(self):
        # print('test_behind_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=-3.0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, 3.0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_aheady_target1(self):
        # print('test_ahead_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=math.pi/2.0, frame='map')
        goal = easy_Odom(x=0.0, y=1.0, v=0.0, heading=math.pi/2.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, -1.0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_aheady_target2(self):
        # print('test_ahead_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=math.pi/2.0, frame='map')
        goal = easy_Odom(x=0.0, y=2.0, v=0.0, heading=math.pi/2.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, -2.0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_aheady_target3(self):
        # print('test_ahead_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=math.pi/2.0, frame='map')
        goal = easy_Odom(x=0.0, y=3.0, v=0.0, heading=math.pi/2.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, -3.0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_behindy_target1(self):
        # print('test_behind_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=math.pi/2.0, frame='map')
        goal = easy_Odom(x=0.0, y=-1.0, v=0.0, heading=math.pi/2.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, 1.0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_behindy_target2(self):
        # print('test_behind_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=math.pi/2.0, frame='map')
        goal = easy_Odom(x=0.0, y=-2.0, v=0.0, heading=math.pi/2.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, 2.0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_behindy_target3(self):
        # print('test_behind_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=math.pi/2.0, frame='map')
        goal = easy_Odom(x=0.0, y=-3.0, v=0.0, heading=math.pi/2.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, 3.0))
        self.assertTrue(is_close(off, 0))
        self.assertTrue(is_close(heading, 0))

    def test_heading_plus(self):
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=0.0, y=0.0, v=0.0, heading=1.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, 0.0))
        self.assertTrue(is_close(off, 0.0))
        self.assertTrue(is_close(heading, -1.0))

    def test_heading_minus(self):
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=0.0, y=0.0, v=0.0, heading=-1.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, 0.0))
        self.assertTrue(is_close(off, 0.0))
        self.assertTrue(is_close(heading, 1.0))

    def test_compound1(self):
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=1.0, y=1.0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, -1.0))
        self.assertTrue(is_close(off, -1.0))
        self.assertTrue(is_close(heading, 0.0))

    def test_compound2(self):
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=1.0, y=1.0, v=0.0, heading=math.pi/4.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, -math.sqrt(2.0)))
        self.assertTrue(is_close(off, 0.0))
        self.assertTrue(is_close(heading, -math.pi/4.0))

    def test_compound3(self):
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=2.0, y=1.0, v=0.0, heading=math.pi/4.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, -1.5*math.sqrt(2.0)))
        self.assertTrue(is_close(off, 0.5*math.sqrt(2.0)))
        self.assertTrue(is_close(heading, -math.pi/4.0))

    def test_compound4(self):
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=0.0, y=1.0, v=0.0, heading=math.pi/4.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, -0.5*math.sqrt(2.0)))
        self.assertTrue(is_close(off, -0.5*math.sqrt(2.0)))
        self.assertTrue(is_close(heading, -math.pi/4.0))

    def test_compound5(self):
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=0.0, y=-1.0, v=0.0, heading=math.pi/4.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, 0.5*math.sqrt(2.0)))
        self.assertTrue(is_close(off, 0.5*math.sqrt(2.0)))
        self.assertTrue(is_close(heading, -math.pi/4.0))

    def test_compound6(self):
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=-2.0, y=-1.0, v=0.0, heading=math.pi/4.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertTrue(is_close(along, 1.5*math.sqrt(2.0)))
        self.assertTrue(is_close(off, -0.5*math.sqrt(2.0)))
        self.assertTrue(is_close(heading, -math.pi/4.0))

    # def test_example_case1(self):
    #     odom_quat = Quaternion()
    #     odom_quat.x = 0.0
    #     odom_quat.y = 0.0
    #     odom_quat.z = -0.321070882395
    #     odom_quat.w = 0.947055166544

    #     goal_quat = Quaternion()
    #     goal_quat.x = 0.0
    #     goal_quat.y = 0.0
    #     goal_quat.z = -0.185333372944
    #     goal_quat.w = 0.982675704835
        
    #     odom_heading = quaternion_to_heading(odom_quat)
    #     goal_heading = quaternion_to_heading(goal_quat)
    #     # print('odom_heading, goal_heading | '+str(odom_heading)+', '+str(goal_heading))
    #     location = easy_Odom(x=1.9925780946, y=2.04027765049, v=0.0, heading=odom_heading, frame='map')
    #     goal = easy_Odom(x=5.84025628099, y=2.20496093823, v=0.0, heading=goal_heading, frame='map')
    #     along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
    #     # print((along, off, heading,))
    #     self.assertTrue(is_close(along, 3.5233694545228))
    #     self.assertTrue(is_close(off, 1.5548683861399))
    #     self.assertTrue(is_close(heading, 0.28089783787221))


if __name__ == '__main__':
    unittest.main()