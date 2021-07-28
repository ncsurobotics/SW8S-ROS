#! /usr/bin/env python
import roslib
roslib.load_manifest('wolf_actions')
import rospy
import actionlib
from std_msgs.msg import Float64, Float32

from wolf_actions.msg import drive_secondsAction, drive_secondsResult

class Drive_Seconds:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('drive_seconds', drive_secondsAction, self.execute, False)
		self.server.start()

	def execute(self, goal):
		rate = rospy.Rate(20)
		initial_time = rospy.get_time()
		lat_pub = rospy.Publisher('wolf_lateral', Float64, queue_size=10)
		while not rospy.is_shutdown() and rospy.get_time() < initial_time + goal.seconds_goal:
			lat_pub.publish(0.1)
			rate.sleep()
		lat_pub.publish(0.0)

		result = drive_secondsResult()
		result.seconds_complete = goal.seconds_goal
		self.server.set_succeeded(result)

if __name__ == '__main__':
	rospy.init_node('drive_seconds_server')
	server = Drive_Seconds()
	rospy.spin()