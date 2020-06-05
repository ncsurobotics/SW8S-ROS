#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import unittest


class PoseTest(unittest.TestCase):
    depth = 0.0

    def pos_callback(self, data):
        self.depth = data.pose.pose.position.z

    def test_depth(self):
        rate = rospy.Rate(20)

        rospy.Subscriber("mavros/global_position/local", Odometry, self.pos_callback)

        success = False
        while not success:
            if self.depth < -2:
                success = True
            rate.sleep()


if __name__ == '__main__':
    import rosunit

    mavros.set_namespace()
    rospy.init_node('test_pose', anonymous=False)
    rosunit.unitrun('wolf_control', 'test_pose', PoseTest)
