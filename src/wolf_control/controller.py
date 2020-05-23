#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64

class Controller:
    
    yaw = 0.0
    depth = 0.0

    def imu_callback(self, data):
        self.yaw = data.z

    def depth_callback(self, data):
        self.depth = data.data

    def __init__(self):
        rospy.init_node('controller', anonymous=False)
        rate = rospy.Rate(20)
        
        rospy.Subscriber("wolf_imu_euler", Vector3, self.imu_callback)
        rospy.Subscriber("wolf_depth", Float64, self.depth_callback)


