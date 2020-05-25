#!/usr/bin/env python
import math
import rospy
import mavros
from std_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import String

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
    yaw_rate = 0.0

    def vertical_callback(self, data):
        self.vertical_rate = value_map(data.data, -1.0, 1.0, 1000, 2000)

    def lateral_callback(self, data):
        self.lateral_rate = value_map(data.data, -1.0, 1.0, 1000, 2000)

    def yaw_callback(self, data):
        self.yaw_rate = value_map(data.data, -1.0, 1.0, 1000, 2000)

    def imu_callback(self, data):
        euler = Vector3()
        euler.x, euler.y, euler.z = quaternion_to_euler(data.orientation.x, data.orientation.y, data.orientation.z,
                                                        data.orientation.w)
        self.imu_pub.publish(euler)

    def __init__(self):
        rospy.init_node('pixhawk', anonymous=False)
        rate = rospy.Rate(20)
        mavros.set_namespace()

        rc = OverrideRCIn()
        self.override_pub = rospy.Publisher(mavros.get_topic("rc", "override"), OverrideRCIn, queue_size=10)
        self.imu_pub = rospy.Publisher("wolf_imu_euler", Vector3, queue_size=10)

        rospy.Subscriber("wolf_vertical", Twist, self.vertical_callback)
        rospy.Subscriber("wolf_lateral", Twist, self.lateral_callback)
        rospy.Subscriber("wolf_yaw", Twist, self.yaw_callback)
        rospy.Subscriber("mavros/imu/data", Imu, self.imu_callback)

        def watchdog_timer():
            vertical_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn, timeout=.05).channels[2]
            if vertical_rate = None:
                print("No message received for 5 miliseconds. Killing thrusters.")
                rc.channels[2] = 1500

            yaw_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn, timeout=.05).channels[3]
            if yaw_rate = None:
                print("No message received for 5 miliseconds. Killing thrusters.")
                rc.channels[3] = 1500
            
            lateral_rate = rospy.wait_for_message(mavros.get_topic("rc", "override"), OverrideRCIn, timeout=.05).channels[4]
            if lateral_rate = None:
                print("No message received for 5 miliseconds. Killing thrusters.")
                rc.channels[4] = 1500

        while not rospy.is_shutdown():
            rc.channels[2] = self.vertical_rate
            rc.channels[3] = self.yaw_rate
            rc.channels[4] = self.lateral_rate
            self.override_pub.publish(rc)
            rate.sleep()


if __name__ == '__main__':
    pixhawk = Pixhawk()
    watchdog_timer()
