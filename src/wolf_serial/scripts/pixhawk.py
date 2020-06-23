#!/usr/bin/env python
import math
import rospy
import mavros
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
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
    val = ostart + (ostop - ostart) * ((value - istart) / (istop - istart))
    if val >= ostop:
        return ostop
    if val <= ostart:
        return ostart

    return val


class Pixhawk:
    pitch_rate = 0.0
    roll_rate = 0.0
    vertical_rate = 0.0
    yaw_rate = 0.0
    forward_rate = 0.0
    strafe_rate = 0.0

    def twist_callback(self, data):
        self.pitch_rate = value_map(data.angular.y, -1.0, 1.0, 1000, 2000) 
        self.roll_rate = value_map(data.angular.x, -1.0, 1.0, 1000, 2000)
        self.vertical_rate = value_map(data.linear.z, -1.0, 1.0, 1000, 2000)
        self.yaw_rate = value_map(data.angular.z, -1.0, 1.0, 1000, 2000)
        self.forward_rate = value_map(data.linear.x, -1.0, 1.0, 1000, 2000)
        self.strafe_rate = value_map(data.linear.y, -1.0, 1.0, 1000, 2000)

    def pose_callback(self, data):
        euler = Vector3()
        orientation = data.pose.orientation
        euler.x, euler.y, euler.z = quaternion_to_euler(orientation.x, orientation.y, orientation.z,
                                                        orientation.w)
        depth = data.pose.position.z
        self.imu_pub.publish(euler)
        self.depth_pub.publish(depth)
        self.yaw_pub.publish(euler.z)

    def __init__(self):
        rospy.init_node('pixhawk', anonymous=False)
        rate = rospy.Rate(20)
        mavros.set_namespace()

        rc = OverrideRCIn()
        self.override_pub = rospy.Publisher(mavros.get_topic("rc", "override"), OverrideRCIn, queue_size=10)
        self.imu_pub = rospy.Publisher("wolf_imu_euler", Vector3, queue_size=10)
        self.yaw_pub = rospy.Publisher("wolf_yaw", Float64, queue_size=10) # Duplicating yaw publishers for now with IMU for PID node
        self.depth_pub = rospy.Publisher("wolf_depth", Float64, queue_size=10)
        rospy.Subscriber("wolf_twist", Twist, self.twist_callback)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_callback)

        while not rospy.is_shutdown():
            rc.channels[0] = self.pitch_rate
            rc.channels[1] = self.roll_rate
            rc.channels[2] = self.vertical_rate
            rc.channels[3] = self.yaw_rate
            rc.channels[4] = self.forward_rate
            rc.channels[5] = self.strafe_rate
            self.override_pub.publish(rc)
            rate.sleep()


if __name__ == '__main__':
    pixhawk = Pixhawk()
