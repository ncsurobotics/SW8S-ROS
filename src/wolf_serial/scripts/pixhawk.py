#!/usr/bin/env python
import rospy
import mavros
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn

vertical_rate = 0.0
lateral_rate = 0.0
rotation_rate = 0.0

def valmap(value, istart, istop, ostart, ostop):
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

def verticalCallback(data):
	global vertical_rate
	vertical_rate = valmap(data.data, -1.0, 1.0, 1000, 2000)

def lateralCallback(data):
	global lateral_rate
	lateral_rate = valmap(data.data, -1.0, 1.0, 1000, 2000)

def rotationCallback(data):
	global rotation_rate
	rotation_rate = valmap(data.data, -1.0, 1.0, 1000, 2000)


def pixhawk():
	global vertical_rate, lateral_rate, rotation_rate

	rospy.init_node('pixhawk', anonymous=False)
	rate = rospy.Rate(20)
	mavros.set_namespace()

	rospy.Subscriber("wolf_vertical", Float64, verticalCallback)
	rospy.Subscriber("wolf_lateral", Float64, lateralCallback)
	rospy.Subscriber("wolf_rotation", Float64, rotationCallback)

	rc = OverrideRCIn()
	override_pub = rospy.Publisher(mavros.get_topic("rc", "override"), OverrideRCIn, queue_size=10)
	while not rospy.is_shutdown():
		rc.channels[2] = vertical_rate
		rc.channels[3] = rotation_rate
		rc.channels[4] = lateral_rate
		override_pub.publish(rc)
		rate.sleep()

if __name__ == '__main__':
	pixhawk()