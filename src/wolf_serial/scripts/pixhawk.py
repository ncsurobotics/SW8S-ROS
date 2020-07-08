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
    print("euler");
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
    print("value map");
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
    isShutdown = True
    
    def twist_callback(self, data):
        print("twist_callback called")
        if self.isShutdown == False:
          self.pitch_rate = value_map(data.angular.y, -1.0, 1.0, 1000, 2000) #how does value map work?
          self.roll_rate = value_map(data.angular.x, -1.0, 1.0, 1000, 2000)
          self.vertical_rate = value_map(data.linear.z, -1.0, 1.0, 1000, 2000)
          self.yaw_rate = value_map(data.angular.z, -1.0, 1.0, 1000, 2000)
          self.forward_rate = value_map(data.linear.x, -1.0, 1.0, 1000, 2000)
          self.strafe_rate = value_map(data.linear.y, -1.0, 1.0, 1000, 2000)
          print(self.pitch_rate)
          print(self.vertical_rate)
        else:
          self.pitch_rate = 1500
          self.roll_rate = 1500
          self.vertical_rate = 1500
          self.yaw_rate = 1500
          self.forward_rate = 1500
          self.strafe_rate = 1500
      
    def pose_callback(self, data):
        print("pose_callback called")
     
        print(self) 
        euler = Vector3()
        orientation = data.pose.orientation
        euler.x, euler.y, euler.z = quaternion_to_euler(orientation.x, orientation.y, orientation.z,
                                                        orientation.w)
        depth = data.pose.position.z
        self.imu_pub.publish(euler)
        self.depth_pub.publish(depth)
        self.yaw_pub.publish(euler.z)
        
    def shutdown_callback(self, data):
        #shutdown code
        print("shutdown_callback called")
        print(data.data)
        if data == 1.0:
          self.isShutdown = True
        
    def __init__(self):
        print("main start")
        rospy.init_node('pixhawk', anonymous=False)
        rate = rospy.Rate(20)
        mavros.set_namespace()

        rc = OverrideRCIn()
        rospy.Subscriber("wolf_meb", Float64, self.shutdown_callback)
        self.override_pub = rospy.Publisher(mavros.get_topic("rc", "override"), OverrideRCIn, queue_size=10)
        self.imu_pub = rospy.Publisher("wolf_imu_euler", Vector3, queue_size=10)
        self.yaw_pub = rospy.Publisher("wolf_yaw", Float64, queue_size=10) # Duplicating yaw publishers for now with IMU for PID node
        self.depth_pub = rospy.Publisher("wolf_depth", Float64, queue_size=10)
        rospy.Subscriber("wolf_twist", Twist, self.twist_callback)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_callback)
        
        
        
        
        while not rospy.is_shutdown():
          rc.channels[0] = 1500
          rc.channels[1] = 1500
          rc.channels[2] = 1500
          rc.channels[3] = 1500
          rc.channels[4] = 1500
          rc.channels[5] = 1500
          self.override_pub.publish(rc)
          rate.sleep()


if __name__ == '__main__':
    pixhawk = Pixhawk()
