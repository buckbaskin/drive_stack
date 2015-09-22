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
 Leader

 This class takes in high level goals, and then develops a path (a series
 of differential poses) for the robot to follow along. Think carrot
 on a stick, where the robot tries to reach the poses being published.
 ''' #pylint: disable=pointless-string-statement

import rospy
