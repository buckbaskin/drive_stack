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

def accel(start_v, end_v, start_w, end_w, time, pub):
    steps = int(math.floor(time/.05))
    dv = (end_v - start_v)/(1.0*steps)
    dw = (end_w - start_w)/(1.0*steps)
    rate = rospy.Rate(1.0/.05)
    twist = Twist()
    rospy.loginfo('accelerating...')
    for i in range(0, steps):
        twist.linear.x = (start_v + dv*steps)
        twist.angular.z = (start_w + dw*steps)
        pub.publish(twist)
        rate.sleep()
    rospy.loginfo('...done accelerating')

def constant(start_v, start_w, time, pub):
    steps = int(math.floor(time/.05))
    rate = rospy.Rate(1.0/.05)
    twist = Twist()
    rospy.loginfo('constant velocity')
    for i in range(0, steps):
        twist.linear.x = start_v
        twist.angular.z = start_w
        pub.publish(twist)
        rate.sleep()
    rospy.loginfo('...done constant velocity')

def main_straight():
    rospy.init_node('velocity_profile')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
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
    accel(0.0,0.5, 0.0,0.0, 1.0,pub)
    # dist = 9.9 sec * 0.5 m/s = 4.9864
    # dist = 4.4864 sec * 0.5 m/s = 2.2432 m
    constant(0.5,0.0,4.4864,pub)
    # dist = 1 sec * 0.5 m/s / 2 = .25m
    accel(0.5,0.0, 0.0,0.0, 1.0,pub)

    # total distance = 5.4864m (18ft)

    # stop
    rospy.loginfo('done with main_straight')
    pub.publish(Twist())
    slow_start.sleep()
    pub.publish(Twist())
    rospy.loginfo('done with main_straight')

def main_turn_in_place():
    rospy.init_node('velocity_profile')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    slow_start = rospy.Rate(1)
    for i in range(5, 0, -1):
        rospy.loginfo(str(i)+'...')
        slow_start.sleep()
    rospy.loginfo('Go')
    # 360deg = 2*pi rad
    # constant dist = 2*pi - .30 rad

    # dist = 1 sec * 0.30 rad/s / 2 = .15 rad
    accel(0.0,0.0, 0.0,0.30, 1.0,pub)
    # dist = 19.94395 sec * 0.3 rad/s = 2*pi - .30 rad
    constant(0.0,0.30, 19.94395,pub)
    # dist = 1 sec * 0.30 rad/s / 2 = .15 rad
    accel(0.0,0.0, 0.30,0.0, 1.0,pub)

    # total distance = 2*pi rad (360 deg)

    # stop
    rospy.loginfo('done with main_turn_in_place')
    pub.publish(Twist())
    slow_start.sleep()
    pub.publish(Twist())
    rospy.loginfo('done with main_turn_in_place')

def main_circle(radius):
    rospy.init_node('velocity_profile')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
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

    dist = 2.0*radius*math.pi
    rospy.loginfo('r: '+str(radius)+' v: '+str(v_max)+' w: '+str(w_max))
    dist_const = dist - v_max
    time_const = dist_const / v_max

    accel(0.0,v_max, 0.0,w_max, 1.0,pub)
    constant(v_max,w_max, time_const, pub)
    accel(v_max,0.0, w_max,0.0, 1.0,pub)

    # stop
    rospy.loginfo('done with main_circle')
    pub.publish(Twist())
    slow_start.sleep()
    pub.publish(Twist())
    rospy.loginfo('done with main_circle')




if __name__ == '__main__':
    main_straight() # runs forward 9ft
    main_turn_in_place() # turns in place 180 degrees
    # radius = 0.5 # meters
    # main_circle(radius) # drives one 360 degree circle at the given radius
