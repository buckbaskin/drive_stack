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
from tf import transformations as tft
from geometry_msgs.msg import Odometry, Twist
from drive_stack.srv import Goal

def quaternion_to_heading(quaternion):
    """
    Converts a quaternion to equivalent Euler yaw/heading
    input: nav_msgs.msg.Quaternion
    output: euler heading in radians
    """
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    yaw = tft.euler_from_quaternion(quaternion)[2]
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

class Driver(object):
    """
    See above
    """
    def __init__(self):
        pass

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
        next = self.lead_goal()
        if (dist(next, odom) < .04):
            # if you are .04 m or less from the goal, move forward
            # NOTE: this might change.
            # for example, you may want to look at the next two points ahead
            #  and see if you want to skip the current one, or if you are ahead
            #  of the current one in the direction that you want to go
            next = self.lead_next()

        # errors along axis "x", off axis "y", heading "theta"
        along, off, heading = calc_errors(odom, next)

        v_error = odom.twist.twist.linear.x - next.twist.twist.linear.x
        omega_error = odom.twist.twist.angular.z - next.twist.twist.angular.z

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
        b = 1
        k = 1

        linear_vel += -k*along
        linear_vel += -b*v_error

        # angular velocity needs to adjust for heading error
        # as well as offset from the desired axis

        # F = -j(theta) - a(dtheta/dt)
        j = 1
        a = 1

        angular_vel += -j*heading_error
        angular_vel += -a*omega_error

        # correction for offset from axis
        # positive error is to the left, so correction is to the right/negative
        # linear-like assumption
        # correction = -c*off , where off is the error off axis
        c = 1
        angular_vel += -c*off

        twist_out = Twist()
        twist_out.linear.x = linear_vel
        twist_out.angular.z = angular_vel
        self.cmd_vel.publish(twist_out)

    def calc_errors(self, location, goal):
        return (self.along_axis_error(location, goal), 
            self.off_axis_error(location, goal), 
            self.heading_error(location, goal),)

    def along_axis_error(self, location, goal):
        # TODO(buckbaskin):
        return 0

    def off_axis_error(self, location, goal):
        # TODO(buckbaskin):
        return 0

    def heading_error(self, location, goal):
        # TODO(buckbaskin):
        return 0

    def run_node(self):
        self.wait_for_services()
        self.init_node()
        
        rt = rospy.rate(10)
        while not rospy.is_shutdown():
            self.cmd_vel.publish(next_cmd())
            rt.sleep()

if __name__ == '__main__':
    d = Driver()
    d.run_node()