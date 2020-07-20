#!/usr/bin/env python
import math
import rospy
import mavros
from geometry_msgs.msg import Vector3, Twist, PoseStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Imu


# takes a value and remaps it from one range into another
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

    # watchdog timer variables
    # (checks to make sure data is flowing into the controller or kills thrusters
    delta_time = 0.0
    initial_time = 0.0

    def twist_callback(self, data):
        self.pitch_rate = value_map(data.angular.y, -1.0, 1.0, 1000, 2000)
        self.roll_rate = value_map(data.angular.x, -1.0, 1.0, 1000, 2000)
        self.vertical_rate = value_map(data.linear.z, -1.0, 1.0, 1000, 2000)
        self.yaw_rate = value_map(data.angular.z, -1.0, 1.0, 1000, 2000)
        self.forward_rate = value_map(data.linear.x, -1.0, 1.0, 1000, 2000)
        self.strafe_rate = value_map(data.linear.y, -1.0, 1.0, 1000, 2000)

    def depth_callback(self, data):
        self.depth_pub.publish(data)

        # check how much time has passed since last update
        self.delta_time = rospy.get_time() - self.initial_time
        self.initial_time = rospy.get_time()

    def imu_callback(self, data):
        self.yaw_pub.publish(data.data)

        # check how much time has passed since last update
        self.delta_time = rospy.get_time() - self.initial_time
        self.initial_time = rospy.get_time()

    def __init__(self):
        rospy.init_node('pixhawk', anonymous=False)
        rate = rospy.Rate(20)
        mavros.set_namespace()

        # time setup
        self.initial_time = rospy.get_time()

        rc = OverrideRCIn()
        self.override_pub = rospy.Publisher(mavros.get_topic("rc", "override"), OverrideRCIn, queue_size=10)

        self.yaw_pub = rospy.Publisher("wolf_yaw", Float64, queue_size=10)
        self.depth_pub = rospy.Publisher("wolf_depth", Float64, queue_size=10)
        rospy.Subscriber("wolf_twist", Twist, self.twist_callback)
        rospy.Subscriber("mavros/global_position/rel_alt", Float64, self.depth_callback)
        rospy.Subscriber("mavros/global_position/compass_hdg", Float64, self.imu_callback)

        while not rospy.is_shutdown():
            rc.channels[0] = self.pitch_rate
            rc.channels[1] = self.roll_rate
            rc.channels[2] = self.vertical_rate
            rc.channels[3] = self.yaw_rate
            rc.channels[4] = self.forward_rate
            rc.channels[5] = self.strafe_rate

            # if no data is coming in, kill thrusters
            if self.delta_time > 0.1:
                rc.channels[0] = 1500
                rc.channels[1] = 1500
                rc.channels[2] = 1500
                rc.channels[3] = 1500
                rc.channels[4] = 1500
                rc.channels[5] = 1500
                rospy.logerr("WATCHDOG TIMER TRIGGERED: SENSOR DATA IS TOO SLOW")

            self.override_pub.publish(rc)
            rate.sleep()


if __name__ == '__main__':
    pixhawk = Pixhawk()
