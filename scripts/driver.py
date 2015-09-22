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
 '''
'''
 Driver

 A Driver is a class that takes in an Odometry
 feed from a Leader class and an Odometry feed
 from its trusted source of localization data
 and then outputs Twist commands for the robot.''' #pylint: disable=pointless-string-statement

import rospy
