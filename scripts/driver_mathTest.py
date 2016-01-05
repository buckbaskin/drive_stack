#!/usr/bin/env python

'''
run the following test with:
python driver_mathTest.py
'''

import unittest

import rospy
from nav_msgs.msg import Odometry

from driver import Driver
from utils import heading_to_quaternion, easy_Odom

class TestDriverCalculations(unittest.TestCase):

    def setUp(self):
        self.driver_obj = Driver()

    def test_equal_odom(self):
        # print('test_equal_odom')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertEqual(along, 0)
        self.assertEqual(off, 0)
        self.assertEqual(heading, 0)

    def test_aheadx_target1(self):
        # print('test_ahead_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=1.0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertEqual(along, 1.0)
        self.assertEqual(off, 0)
        self.assertEqual(heading, 0)

    def test_aheadx_target2(self):
        # print('test_ahead_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=2.0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertEqual(along, 2.0)
        self.assertEqual(off, 0)
        self.assertEqual(heading, 0)

    def test_aheadx_target3(self):
        # print('test_ahead_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=3.0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertEqual(along, 3.0)
        self.assertEqual(off, 0)
        self.assertEqual(heading, 0)

    def test_behindx_target1(self):
        # print('test_behind_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=-1.0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertEqual(along, -1.0)
        self.assertEqual(off, 0)
        self.assertEqual(heading, 0)

    def test_behindx_target2(self):
        # print('test_behind_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=-2.0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertEqual(along, -2.0)
        self.assertEqual(off, 0)
        self.assertEqual(heading, 0)

    def test_behindx_target3(self):
        # print('test_behind_target')
        location = easy_Odom(x=0, y=0, v=0.0, heading=0.0, frame='map')
        goal = easy_Odom(x=-3.0, y=0, v=0.0, heading=0.0, frame='map')
        along, off, heading = self.driver_obj.calc_errors(location=location, goal=goal)
        self.assertEqual(along, -3.0)
        self.assertEqual(off, 0)
        self.assertEqual(heading, 0)

if __name__ == '__main__':
    unittest.main()