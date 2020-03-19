#! /usr/bin/env python

import roslib
roslib.load_manifest('wolf_actions')
import rospy
import actionlib

from wolf_actions.msg import submerge_secondsAction, submerge_secondsGoal

if __name__ == '__main__':
	rospy.init_node('dummy_client')

	client = actionlib.SimpleActionClient('submerge_seconds', submerge_secondsAction)
	client.wait_for_server()

	goal = submerge_secondsGoal()
	goal.seconds_goal = 5.0
	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(10.0))