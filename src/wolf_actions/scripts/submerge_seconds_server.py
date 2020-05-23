#! /usr/bin/env python
import roslib

roslib.load_manifest('wolf_actions')
import rospy
import actionlib
from geometry_msgs.msg import Twist
from wolf_actions.msg import submerge_secondsAction, submerge_secondsResult


class Submerge_Seconds:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('submerge_seconds', submerge_secondsAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        twist = Twist()
        rate = rospy.Rate(20)
        initial_time = rospy.get_time()
        twist_pub = rospy.Publisher('wolf_twist', Twist, queue_size=10)
        while not rospy.is_shutdown() and rospy.get_time() < initial_time + goal.seconds_goal:
            twist.linear.z = -0.2
            twist_pub.publish(twist)
            rate.sleep()
        twist.linear.z = 0.0
        twist_pub.publish(twist)

        result = submerge_secondsResult()
        result.seconds_complete = goal.seconds_goal
        self.server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('submerge_seconds_server')
    server = Submerge_Seconds()
    rospy.spin()
