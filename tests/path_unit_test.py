#!/usr/bin/env python
PKG = 'drive_stack'
NAME = 'test_unit_path'

import sys
import unittest
print sys.path
sys.path.append('/home/buck/ros_workspace/src/drive_stack/scripts')
# from drive_stack.scripts import Path
import path

class TestPathUnit(unittest.TestCase):

    def __init__(self, *args):
        super(TestPathUnit, self).__init__('test_example')
        self.p = path.Path()

    def test_example(self):
        self.assertEquals(1, 1, "1 != 1")

def run_tests():
    import rosunit
    rosunit.unitrun(PKG, NAME, TestPathUnit)


if __name__ == '__main__':
    run_tests()
