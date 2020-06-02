#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class Oracle:
    depth = 0.0

    def pos_callback(self, data):
        self.depth = data.pose.position.z

    def __init__(self):
        rospy.init_node('oracle', anonymous=False)
        rate = rospy.Rate(20)
        mavros.set_namespace()

        pose = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pos_callback)
        pub = rospy.Publisher("wolf_success", Bool, queue_size=10)
        pub.publish(False)

        while not rospy.is_shutdown():
            if self.depth < -2:
                pub.publish(True)
            rate.sleep()


if __name__ == '__main__':
    oracle = Oracle()
