#!/usr/bin/env python

import rospy
import mavros
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Float64
import unittest


class PixhawkTest(unittest.TestCase):

    def test_vertical_rate(self):
        pub = rospy.Publisher("wolf_vertical", Float64, queue_size=10)

        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)

        pub.publish(0.1)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(vertical_rate, 1550.0, "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(vertical_rate, 1550))

        pub.publish(-0.1)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(vertical_rate, 1450.0, "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(vertical_rate, 1450))

        pub.publish(1000)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(vertical_rate, 2000.0, "PIXHAWK NODE: inputting rate > 1.0 results in bad RC rate")

        pub.publish(-1000)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(vertical_rate, 1000.0, "PIXHAWK NODE: inputting rate < 1.0 results in bad RC rate")

    def test_lateral_rate(self):
        pub = rospy.Publisher("wolf_lateral", Float64, queue_size=10)

        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)
        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)

        pub.publish(0.1)
        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[4]
        self.assertEqual(lateral_rate, 1550.0, "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(lateral_rate, 1550))

        pub.publish(-0.1)
        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[4]
        self.assertEqual(lateral_rate, 1450.0, "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(lateral_rate, 1450))

        pub.publish(1000)
        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[4]
        self.assertEqual(lateral_rate, 2000.0, "PIXHAWK NODE: inputting rate > 1.0 results in bad RC rate")

        pub.publish(-1000)
        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[4]
        self.assertEqual(lateral_rate, 1000.0, "PIXHAWK NODE: inputting rate < 1.0 results in bad RC rate")


    def test_rotation_rate(self):
        pub = rospy.Publisher("wolf_rotation", Float64, queue_size=10)

        rotation_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)
        rotation_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)

        pub.publish(0.1)
        rotation_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[3]
        self.assertEqual(rotation_rate, 1550.0, "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(rotation_rate, 1550))

        pub.publish(-0.1)
        rotation_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[3]
        self.assertEqual(rotation_rate, 1450.0, "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(rotation_rate, 1450))

        pub.publish(1000)
        rotation_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[3]
        self.assertEqual(rotation_rate, 2000.0, "PIXHAWK NODE: inputting rate > 1.0 results in bad RC rate")

        pub.publish(-1000)
        rotation_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[3]
        self.assertEqual(rotation_rate, 1000.0, "PIXHAWK NODE: inputting rate < 1.0 results in bad RC rate")

if __name__ == '__main__':
    import rosunit
    mavros.set_namespace()
    rospy.init_node('test_pixhawk', anonymous=False)
    rosunit.unitrun('wolf_serial', 'test_pixhawk', PixhawkTest)
