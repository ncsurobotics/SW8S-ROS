#!/usr/bin/env python

import rospy
import mavros
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Twist
import unittest


class PixhawkTest(unittest.TestCase):

    def test_vertical_rate(self):
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)

        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)
        twist_out = Twist()

        twist_out.linear.z = 0.1
        pub.publish(twist_out)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(vertical_rate, 1550.0,
                         "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(vertical_rate,
                                                                                                      1550))

        twist_out.linear.z = -0.1
        pub.publish(twist_out)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(vertical_rate, 1450.0,
                         "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(vertical_rate,
                                                                                                      1450))

        twist_out.linear.z = 1000
        pub.publish(twist_out)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(vertical_rate, 2000.0, "PIXHAWK NODE: inputting rate > 1.0 results in bad RC rate")

        twist_out.linear.z = -1000
        pub.publish(twist_out)
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(vertical_rate, 1000.0, "PIXHAWK NODE: inputting rate < 1.0 results in bad RC rate")

    def test_lateral_rate(self):
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)

        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)
        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)
        twist_out = Twist()

        twist_out.linear.x = 0.1
        pub.publish(twist_out)
        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[4]
        self.assertEqual(lateral_rate, 1550.0,
                         "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(lateral_rate,
                                                                                                      1550))

        twist_out.linear.x = -0.1
        pub.publish(twist_out)
        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[4]
        self.assertEqual(lateral_rate, 1450.0,
                         "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(lateral_rate,
                                                                                                      1450))

        twist_out.linear.x = 1000
        pub.publish(twist_out)
        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[4]
        self.assertEqual(lateral_rate, 2000.0, "PIXHAWK NODE: inputting rate > 1.0 results in bad RC rate")

        twist_out.linear.x = -1000
        pub.publish(twist_out)
        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[4]
        self.assertEqual(lateral_rate, 1000.0, "PIXHAWK NODE: inputting rate < 1.0 results in bad RC rate")

    def test_yaw_rate(self):
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)

        yaw_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)
        yaw_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)
        twist_out = Twist()

        twist_out.angular.z = 0.1
        pub.publish(twist_out)
        yaw_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[3]
        self.assertEqual(yaw_rate, 1550.0,
                         "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(yaw_rate, 1550))

        twist_out.angular.z = -0.1
        pub.publish(twist_out)
        yaw_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[3]
        self.assertEqual(yaw_rate, 1450.0,
                         "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(yaw_rate, 1450))

        twist_out.angular.z = 1000
        pub.publish(twist_out)
        yaw_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[3]
        self.assertEqual(yaw_rate, 2000.0, "PIXHAWK NODE: inputting rate > 1.0 results in bad RC rate")

        twist_out.angular.z = -1000
        pub.publish(twist_out)
        yaw_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[3]
        self.assertEqual(yaw_rate, 1000.0, "PIXHAWK NODE: inputting rate < 1.0 results in bad RC rate")

    def test_mixed_rates(self):
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)

        yaw_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)
        yaw_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn)
        twist_out = Twist()

        twist_out.angular.z = 0.1
        twist_out.linear.x = 0.1
        twist_out.linear.z = 0.1
        pub.publish(twist_out)
        yaw_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[3]
        lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[4]
        vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn).channels[2]
        self.assertEqual(yaw_rate, 1550.0,
                         "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(yaw_rate, 1550))
        self.assertEqual(lateral_rate, 1550.0,
                         "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(lateral_rate,
                                                                                                      1550))
        self.assertEqual(vertical_rate, 1550.0,
                         "PIXHAWK NODE: RC rate does not match sent rate, was {} should be {}".format(vertical_rate,
                                                                                                      1550))


if __name__ == '__main__':
    import rosunit

    mavros.set_namespace()
    rospy.init_node('test_pixhawk', anonymous=False)
    rosunit.unitrun('wolf_serial', 'test_pixhawk', PixhawkTest)
