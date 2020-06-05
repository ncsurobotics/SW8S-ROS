#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import unittest


class PoseTest(unittest.TestCase):
    depth = 0.0
    tolerance = 0.1  # what is the percentage of error that is allowable
    hold_time = 1  # how long (in seconds) should the error be held within tolerance
    rate = 20

    def pos_callback(self, data):
        self.depth = data.pose.pose.position.z

    def test_depth(self):
        target_depth = -3
        error_margin = target_depth * self.tolerance
        initial_time = rospy.get_time()

        rate = rospy.Rate(self.rate)

        rospy.Subscriber("mavros/global_position/local", Odometry, self.pos_callback)

        success = False
        ticks = 0  # counts the number of cycles that the tolerance has been met
        target_ticks = self.rate * self.hold_time
        while not success:
            if target_depth - abs(error_margin) < self.depth < target_depth + abs(error_margin):
                ticks += 1
                if ticks >= target_ticks:
                    rospy.logdebug("Took {} seconds to complete Depth Test, had {}% error"
                                   .format(rospy.get_time() - initial_time,
                                           (abs(self.depth - target_depth) / abs(target_depth)) * 100))
                    success = True
            else:
                ticks = 0
            rate.sleep()


if __name__ == '__main__':
    import rosunit

    mavros.set_namespace()
    rospy.init_node('test_pose', anonymous=False, log_level=rospy.DEBUG)
    rosunit.unitrun('wolf_control', 'test_pose', PoseTest)
