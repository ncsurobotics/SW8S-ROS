#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64

class Controller:   
    twist_state = Twist()
#    yaw_control_out = 0.0
    depth_control_out = 0.0

    def twist_callback(self, twist):
        self.depth_set_pub.publish(twist.linear.z)

    def depth_control_callback(self, controller_output):
        self.depth_control_out = controller_output.data

    def __init__(self):
        rospy.init_node('controller', anonymous=False)
        rate = rospy.Rate(20)
        rospy.loginfo("controller constructor reached")
        
#        rospy.Subscriber("wolf_imu_euler", Vector3, lambda data: self.yaw_state=data.z)
#        rospy.Subscriber("wolf_depth", Float64, lambda data: self.depth_state=data.data)
#        self.yaw_state_pub = rospy.Publisher("yaw_state", Float64, queue_size=10)
#        self.depth_state_pub = rospy.Publisher("depth_state", Float64, queue_size=10)


        rospy.Subscriber("twist_setpoint", Twist, self.twist_callback)
#        self.yaw_set_pub = rospy.Publisher("yaw_setpoint", Float64, queue_size=10)
        self.depth_set_pub = rospy.Publisher("depth_setpoint", Float64, queue_size=10)

#        rospy.Subscriber("yaw_control_output", Float64, lambda data: self.yaw_control_out=data.data)
        rospy.Subscriber("depth_control_output", Float64, self.depth_control_callback)
        self.twist_pub = rospy.Publisher("wolf_twist", Twist, queue_size=10)

        while not rospy.is_shutdown():
            twist_out = Twist()
            twist_out.linear.z = self.depth_control_out
            self.twist_pub.publish(twist_out)
            rate.sleep()

if __name__ == '__main__':
    controller = Controller()
