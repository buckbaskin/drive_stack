PKG = 'drive_stack'
NAME = 'test_unit_path'

import sys
import unittest
from drive_stack.scripts.path import Path

class TestPathUnit(unittest.TestCase):

    def __init__(self):
        self.p = Path()

    def test_example(self):
        self.assertEquals(1, 1, "1 != 1")

def run_tests():
	import rosunit
    rosunit.unitrun(PKG, NAME, TestPathUnit)

if __name__ == '__main__':
    run_tests()