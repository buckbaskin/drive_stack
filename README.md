# drive_stack
lang:ROS/Python/C++ End to end solution for driving a robot based on position.
[![Build Status](https://travis-ci.org/buckbaskin/drive_stack.svg?branch=master)](https://travis-ci.org/buckbaskin/drive_stack)

## Components

The overall interface is:

- listen on msg/odom for current position
- listen on msg/lead/desired for current desired position
- publish on msg/cmd_vel to control the robot

### Path

Maintains the high level list of position goals for the robot to attain in map space. 

### Leader

Creates and updates the lower level list of interim position sub-goals for the robot to try to match to reach between Path goals.

### Driver

Outputs a velocity command based on a current position and the desired next position from a Leader to try and match the two.

For everything to run properly, run the following:
catkin_make --only-pkg-with-deps drive_stack
