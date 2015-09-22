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
 Path

 This is a class that generates high level goals in the map frame
 for a robot to achieve. It is designed to sync with a 
 Leader node that takes the published goals, and then generates
 Odometry targets between them for the robot to follow (think carrot
 on a stick).
 ''' #pylint: disable=pointless-string-statement

import rospy
