#!/usr/bin/env python
import math
import rospy
import mavros
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from mavros_msgs.msg import OverrideRCIn


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z


def value_map(value, istart, istop, ostart, ostop):
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


class Pixhawk:
    vertical_rate = 0.0
    lateral_rate = 0.0
    rotation_rate = 0.0

    def vertical_callback(self, data):
        self.vertical_rate = value_map(data.data, -1.0, 1.0, 1000, 2000)
        if data.data >= 1.0:
            self.vertical_rate = 2000
        if data.data <= -1.0:
            self.vertical_rate = 1000

    def lateral_callback(self, data):
        self.lateral_rate = value_map(data.data, -1.0, 1.0, 1000, 2000)
        if data.data >= 1.0:
            self.lateral_rate = 2000
        if data.data <= -1.0:
            self.lateral_rate = 1000

    def rotation_callback(self, data):
        self.rotation_rate = value_map(data.data, -1.0, 1.0, 1000, 2000)
        if data.data >= 1.0:
            self.rotation_rate = 2000
        if data.data <= -1.0:
            self.rotation_rate = 1000

    def imu_callback(self, data):
        euler = Vector3()
        euler.x, euler.y, euler.z = quaternion_to_euler(data.orientation.x, data.orientation.y, data.orientation.z,
                                                        data.orientation.w)
        self.imu_pub.publish(euler)

    def __init__(self):
        rospy.init_node('pixhawk', anonymous=False)
        rate = rospy.Rate(20)
        mavros.set_namespace()

        print(mavros.get_topic("rc", "override"))
        rc = OverrideRCIn()
        self.override_pub = rospy.Publisher(mavros.get_topic("rc", "override"), OverrideRCIn, queue_size=10)
        self.imu_pub = rospy.Publisher("wolf_imu_euler", Vector3, queue_size=10)

        rospy.Subscriber("wolf_vertical", Float64, self.vertical_callback)
        rospy.Subscriber("wolf_lateral", Float64, self.lateral_callback)
        rospy.Subscriber("wolf_rotation", Float64, self.rotation_callback)
        rospy.Subscriber("mavros/imu/data", Imu, self.imu_callback)

        while not rospy.is_shutdown():
            rc.channels[2] = int(self.vertical_rate)
            rc.channels[3] = int(self.rotation_rate)
            rc.channels[4] = int(self.lateral_rate)
            rc.channels = np.array(rc.channels).astype(np.uint16)
            self.override_pub.publish(rc)
            rate.sleep()


if __name__ == '__main__':
    pixhawk = Pixhawk()
