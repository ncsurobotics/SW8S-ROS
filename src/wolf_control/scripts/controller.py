#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64


class Controller:
    twist_state = Twist()
    yaw_control_out = 0.0
    depth_control_out = 0.0
    yaw = 0.0

    def twist_callback(self, twist):
        self.depth_set_pub.publish(twist.linear.z)
        self.yaw_set_pub.publish(twist.angular.z)

    def imu_callback(self, orientation):
        self.yaw = orientation.z

    def depth_control_callback(self, controller_output):
        self.depth_control_out = controller_output.data

    def yaw_control_callback(self, controller_output):
        self.yaw_control_out = controller_output.data

    def __init__(self):
        rospy.init_node('controller', anonymous=False)
        rate = rospy.Rate(20)

        rospy.Subscriber("wolf_imu_euler", Vector3, self.imu_callback)
        rospy.Subscriber("wolf_twist_setpoint", Twist, self.twist_callback)
        self.yaw_set_pub = rospy.Publisher("wolf_control/yaw_setpoint", Float64, queue_size=10)
        self.depth_set_pub = rospy.Publisher("wolf_control/depth_setpoint", Float64, queue_size=10)

        rospy.Subscriber("wolf_control/yaw_output", Float64, self.yaw_control_callback)
        rospy.Subscriber("wolf_control/depth_output", Float64, self.depth_control_callback)
        self.twist_pub = rospy.Publisher("wolf_twist", Twist, queue_size=10)

        while not rospy.is_shutdown():
            twist_out = Twist()
            twist_out.linear.z = self.depth_control_out
            twist_out.angular.z = self.yaw_control_out
            self.twist_pub.publish(twist_out)
            rate.sleep()


if __name__ == '__main__':
    controller = Controller()
