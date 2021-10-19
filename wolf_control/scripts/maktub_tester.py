#!/usr/bin/env python

import rospy
import unittest
from std_msgs.msg import String
class MaktubTest(unittest.TestCase):
    def testMaktub(self):
        limit = 300;
        startTime = rospy.Time.now()
        while True:
            message: String = rospy.wait_for_message("/maktub/test_log", String)
            if message.data == "Test successfully completed":
                return
            if (rospy.Time.now() - startTime).to_sec() > 300:
                self.fail("Time has run out to complete the test!")

if __name__ == '__main__':
    import rosunit
    rospy.init_node('maktub_tester', anonymous=False)
    rosunit.unitrun("wolf_control", "maktub_tester", MaktubTest)