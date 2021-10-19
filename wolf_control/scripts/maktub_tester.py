#!/usr/bin/env python

from time import time
import rospy
import unittest
from std_msgs.msg import String
class MaktubTest(unittest.TestCase):
    def testMaktub(self):
        limit = 300;
        startTime = rospy.Time.now()
        while not rospy.is_shutdown():
            message: String = rospy.wait_for_message("/maktub/test_log", String, timeout=limit)
            if message.data == "Test successfully completed":
                return

if __name__ == '__main__':
    import rosunit
    rospy.init_node('maktub_tester', anonymous=False)
    rosunit.unitrun("wolf_control", "maktub_tester", MaktubTest)