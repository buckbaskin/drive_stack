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
 
Poke

Create this class to request data from various service interfaces
to test for functionality. Interactive.
'''


import rospy
from nav_msgs.msg import Odometry
import drive_stack

from rostype import returns

class Poke(object):
	def __init__(self):
		self.error_log = []
		self.test_count = 0
		self.test_passes = 0
		self.test_failures = 0

	def poke(channel):
		pass

	def test(channel = None):

		if channel is None:
			channel = input('Please enter the channel to test')
		out = poke(channel)
		input_from_poke = out[1]
		output_from_poke = out[0]
		print output_from_poke
		success = input('Was the service call successful?')
		if not success:
			self.test_failures += 1
			self.error_log.append((channel, input_from_poke, output_from_poke, 'User rejected data',))
		else:
			self.test_passes += 1

	def run_test():
		self.error_log = []
		channels = []
		channels.append('/test')
		for channel in channels:
			self.test_count += 1
			test(channel)

		print 'TEST SUMMARY: '
		print str(self.test_count)+' tests run'
		print str(self.test_passes)+' tests passed'
		print str(self.test_failures)+' tests failed'

		if len(self.error_log):
			'TEST ERRORS: '
		for error in self.error_log:
			print (str(error[3])+'\n'+': tried '+str(error[0])+' with data '+str(error[1])+
				'.\n Got '+str(error[2]))
