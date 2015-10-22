PKG = 'drive_stack'
NAME = 'test_integration_path'

import sys
import unittest
from drive_stack.scripts.path import Path
from drive_stack.srv import *

class TestPathIntegration(unittest.TestCase):

    def __init__(self):
        self.p = Path()
        p.run_server()

    def test_path_goal_service(self):
        rospy.wait_for_service('/path/goal')
        s = rospy.ServiceProxy('/path/goal', Goal)
        resp = s().goal
        resp2 = s.call(GoalRequest()).goal
        # check that response options are equal
        self.assetEquals(resp.pose.pose.position.x, resp2.pose.pose.position.x)
        self.assetEquals(resp.pose.pose.position.y, resp2.pose.pose.position.y)
        # check that the response is the expected position
        self.assetEquals(resp.pose.pose.position.x, 0)
        self.assetEquals(resp.pose.pose.position.y, 1)

    def test_path_start_service(self):
        rospy.wait_for_service('/path/start')
        s = rospy.ServiceProxy('/path/start', Goal)
        resp = s().goal
        resp2 = s.call(GoalRequest()).goal
        # check that response options are equal
        self.assetEquals(resp.pose.pose.position.x, resp2.pose.pose.position.x)
        self.assetEquals(resp.pose.pose.position.y, resp2.pose.pose.position.y)
        # check that the response is the expected position
        self.assetEquals(resp.pose.pose.position.x, 0)
        self.assetEquals(resp.pose.pose.position.y, 0)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, NAME, TestPathIntegration)