#!/usr/bin/env python

import rospy
import mavros
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Float64
import unittest


class PixhawkTest(unittest.TestCase):

    def verticalRateTest(self):
        pub = rospy.Publisher("wolf_vertical", Float64, queue_size=10)

        pub.publish(0.1)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(vertical_rate, 1550.0, "PIXHAWK NODE: RC rate does not match sent rate")

        pub.publish(-0.1)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(vertical_rate, 1450.0, "PIXHAWK NODE: RC rate does not match sent rate")

        pub.publish(1000)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(vertical_rate, 2000.0, "PIXHAWK NODE: inputting rate > 1.0 results in bad RC rate")

        pub.publish(-1000)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(vertical_rate, 1000.0, "PIXHAWK NODE: inputting rate < 1.0 results in bad RC rate")


if __name__ == '__main__':
    import rosunit

    rosunit.unitrun('wolf_serial', 'test_pixhawk', PixhawkTest)
