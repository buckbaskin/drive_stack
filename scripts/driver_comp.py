#!/usr/bin/env python
"""
Competition Driver
Implementation of the driver for The Competition

process_position is the key method here
"""

import driver_pseudolinear
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from drive_stack.srv import Goal

class CompetitionDriver(driver_pseudolinear.PseudoLinearDriver):
    def __init__(self):
        super(CompetitionDriver, self).__init__()
        self.silent = False

    def wait_for_services(self, node_name='competition_driver'):
        super(CompetitionDriver, self).wait_for_services(node_name)
        rospy.loginfo('waiting for services\n')

    def init_node(self):
        """
        Start the pub/sub portion of the ROS node
        """
        super(CompetitionDriver, self).init_node()
        # use this to set the proper channels for the Driver to listen to for data
        self.position = rospy.Subscriber('/odom', Odometry, self.process_position)

        # use this to set the proper channels for the Driver to publish on
        self.last_pose_data = rospy.Publisher('/last_pose_data', Odometry, queue_size=1)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

if __name__ == '__main__':
    d = CompetitionDriver()
    d.run_node()
