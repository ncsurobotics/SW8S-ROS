#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64
import numpy as np
'''
Lateral input: x and y in "absolute" (IMU) frame
Rotational input: IMU frame degrees
This needs to be transformed into "robot" frame (X is forward)
At each timestep:
    Rotate input x/y into robot frame
    Publish:
        linear:
            x: rotated input
            y: rotated input
            z: PID output with input setpoint (no transformation)
        angular
            x: 0
            y: 0
            z: PID output with input setpoint (no transformation)
Questions for Ore:
    - Absolute vs. Relative
    - Mutex on goal pose publisher?
    - tf transformations
    
'''

class Controller:   
    twist_state = Twist()
    yaw_control_out = 0.0
    depth_control_out = 0.0
    yaw = 0.0
    max_lateral_speed = 0.4
    imu_x = 0.0
    imu_y = 0.0
    robot_x = 0.0
    robot_y = 0.0
    
    def twist_callback(self, twist):
        self.depth_set_pub.publish(twist.linear.z)
        self.yaw_set_pub.publish(twist.angular.z)
        self.imu_x = twist.linear.x
        self.imu_y = twist.linear.y



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
            
            #Perform coordinate transform on absolute inputs
            
            theta = np.deg2rad(-self.yaw)
            cos = np.cos(theta)
            sin = np.sin(theta)
            rotation = np.array([[cos, -sin], [sin, cos]])
            imu_vec = np.array([self.imu_x, self.imu_y])
            robot_vec = rotation.dot(imu_vec)
            mag = np.linalg.norm(robot_vec)
            if mag > 0.01:
                robot_vec_capped = self.max_lateral_speed * robot_vec / np.linalg.norm(robot_vec)
                self.robot_x, self.robot_y = robot_vec_capped
            else:
                self.robot_x = 0
                self.robot_y = 0

            twist_out = Twist()
            twist_out.linear.x = self.robot_x
            twist_out.linear.y = self.robot_y
            twist_out.linear.z = self.depth_control_out
            twist_out.angular.z = self.yaw_control_out
            self.twist_pub.publish(twist_out)
            rate.sleep()

if __name__ == '__main__':
    controller = Controller()
