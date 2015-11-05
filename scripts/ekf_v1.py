# #!/usr/bin/env python

'''
Copyright 2015 William Baskin

/*****************************************
 LICENSE SUMMARY

 This package is licensed under the
    MIT License. Please see the LICENSE.md
    file in the root folder for the
    complete license.

 *****************************************/

EKF v1

first round attempt at implementing an EKF based on
algorithm(s) presented in:

Probabilistic Robotics by Thrun, Burgard, Fox circa 2006

see ch. 7

first iteration organized as:
class Ekf - the actual brains behind the operation

class EkfNode - provides interface to ros to expose ekf methods
'''

class EKF(object):
	pass