#!/usr/bin/env python
import rospy
import mavros
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn


def value_map(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


class Pixhawk:
    vertical_rate = 0.0
    lateral_rate = 0.0
    rotation_rate = 0.0

    def vertical_callback(self, data):
        self.vertical_rate = value_map(data.data, -1.0, 1.0, 1000, 2000)

    def lateral_callback(self, data):
        self.lateral_rate = value_map(data.data, -1.0, 1.0, 1000, 2000)

    def rotation_callback(self, data):
        self.rotation_rate = value_map(data.data, -1.0, 1.0, 1000, 2000)

    def __init__(self):
        rospy.init_node('pixhawk', anonymous=False)
        rate = rospy.Rate(20)
        mavros.set_namespace()

        rospy.Subscriber("wolf_vertical", Float64, self.vertical_callback)
        rospy.Subscriber("wolf_lateral", Float64, self.lateral_callback)
        rospy.Subscriber("wolf_rotation", Float64, self.rotation_callback)

        rc = OverrideRCIn()
        override_pub = rospy.Publisher(mavros.get_topic("rc", "override"), OverrideRCIn, queue_size=10)
        while not rospy.is_shutdown():
            rc.channels[2] = self.vertical_rate
            rc.channels[3] = self.rotation_rate
            rc.channels[4] = self.lateral_rate
            override_pub.publish(rc)
            rate.sleep()


if __name__ == '__main__':
    pixhawk = Pixhawk()
