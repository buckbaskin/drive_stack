#!/usr/bin/env python

"""
Drive the robot in one of 3 geometric shapes (line, point, circle)
Do so with smooth ramping (accel method) and then constant velocity,
then smooth ramping down (accel again).

It's probably advised to only run one of the main_ methods at the bottom.
"""

import rospy
import math
from geometry_msgs.msg import Twist

class VelocityProfileExecution:
    def __init__(self):
        rospy.init_node('velocity_profile')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.path = rospy.get_param('~path',1)

    def accel(self, start_v, end_v, start_w, end_w, time):
        steps = int(math.floor(time/.05))
        dv = (end_v - start_v)/(1.0*steps)
        dw = (end_w - start_w)/(1.0*steps)
        rate = rospy.Rate(1.0/.05)
        twist = Twist()
        rospy.loginfo('accelerating...')
        for i in range(0, steps):
            twist.linear.x = (start_v + dv*i)
            twist.angular.z = (start_w + dw*i)
            self.pub.publish(twist)
            if rospy.is_shutdown():
                rospy.loginfo('break node loop')
                break
            rate.sleep()
        rospy.loginfo('...done accelerating')

    def constant(self, start_v, start_w, time):
        steps = int(math.floor(time/.05))
        rate = rospy.Rate(1.0/.05)
        twist = Twist()
        rospy.loginfo('constant velocity')
        for i in range(0, steps):
            twist.linear.x = start_v
            twist.angular.z = start_w
            self.pub.publish(twist)
            if rospy.is_shutdown():
                rospy.loginfo('break node loop')
                break
            rate.sleep()
        rospy.loginfo('...done constant velocity')

    def main_straight(self):
        slow_start = rospy.Rate(1)
        for i in range(5, 0, -1):
            rospy.loginfo(str(i)+'...')
            slow_start.sleep()
        rospy.loginfo('Go')
        # 18ft = 5.4864m
        # 9 ft = 2.7432m
        # constant dist = 4.9864 m
        # constant dist = 2.2432 m

        # dist = 1 sec * 0.5 m/s / 2 = .25m
        self.accel(0.0,0.5, 0.0,0.0, 1.0)
        # dist = 9.9 sec * 0.5 m/s = 4.9864
        # dist = 4.4864 sec * 0.5 m/s = 2.2432 m
        self.constant(0.5,0.0,4.4864)
        # dist = 1 sec * 0.5 m/s / 2 = .25m
        self.accel(0.5,0.0, 0.0,0.0, 1.0)

        # total distance = 5.4864m (18ft)

        # stop
        rospy.loginfo('done with main_straight')
        self.pub.publish(Twist())
        slow_start.sleep()
        self.pub.publish(Twist())
        rospy.loginfo('done with main_straight')

    def main_turn_in_place(self):
        slow_start = rospy.Rate(1)
        for i in range(5, 0, -1):
            rospy.loginfo(str(i)+'...')
            slow_start.sleep()
        rospy.loginfo('Go')
        # 360deg = 2*pi rad
        # constant dist = 2*pi - .30 rad
        # 180 deg = pi rad
        # constant dist = pi - .30 rad

        # dist = 1 sec * 0.30 rad/s / 2 = .15 rad
        self.accel(0.0,0.0, 0.0,0.30, 1.0)
        # dist = 19.94395 sec * 0.3 rad/s = 2*pi - .30 rad
        # dist = 9.4719 sec * 0.3 rad/s = pi - .30 rad
        self.constant(0.0,0.30, 9.4719)
        # dist = 1 sec * 0.30 rad/s / 2 = .15 rad
        self.accel(0.0,0.0, 0.30,0.0, 1.0)

        # total distance = 2*pi rad (360 deg)

        # stop
        rospy.loginfo('done with main_turn_in_place')
        self.pub.publish(Twist())
        slow_start.sleep()
        self.pub.publish(Twist())
        rospy.loginfo('done with main_turn_in_place')

    def main_circle(self, radius):
        slow_start = rospy.Rate(1)
        for i in range(5, 0, -1):
            rospy.loginfo(str(i)+'...')
            slow_start.sleep()
        rospy.loginfo('Go')
        # v = wr
        v_max = .5
        w_max = v_max/radius
        if w_max > .3:
            w_max = .3
            v_max = w_max*radius

        dist = abs(2.0*radius*math.pi)
        rospy.loginfo('r: '+str(radius)+' v: '+str(v_max)+' w: '+str(w_max))
        dist_const = dist - v_max
        time_const = dist_const / v_max

        self.accel(0.0,v_max, 0.0,w_max, 1.0)
        self.constant(v_max,w_max, time_const)
        self.accel(v_max,0.0, w_max,0.0, 1.0)

        # stop
        rospy.loginfo('done with main_circle')
        self.pub.publish(Twist())
        slow_start.sleep()
        self.pub.publish(Twist())
        rospy.loginfo('done with main_circle')




if __name__ == '__main__':
    # Create a VelocityProfileExecution object.
    velProfEx = VelocityProfileExecution()

    # Based on the path parameter (append " _path:={1-3}" to the command line call)
    # do one of the following actions

    # Drive in a CW circle of radius -0.5 m
    if velProfEx.path == 1:
        radius = -0.5 # meters
        velProfEx.main_circle(radius) # drives one 360 degree circle at the given radius

    # Drive in a CCW circle of radius 1 m
    if velProfEx.path == 2:
        radius = 1 # meters
        velProfEx.main_circle(radius) # drives one 360 degree circle at the given radius

    # Drive forward 9 ft, rotate CCW 180 degrees, drive 9 ft, rotate CCW 180 degrees
    if velProfEx.path == 3:
        velProfEx.main_straight() # runs forward 9ft    
        velProfEx.main_turn_in_place() # turns in place 180 degrees
        velProfEx.main_straight() # runs forward 9ft    
        velProfEx.main_turn_in_place() # turns in place 180 degrees
