#!/usr/bin/env python

import rospy # imports rospy

import tf_conversions #interface that specify conversion between tf2 data types ad external library data types

import tf2_ros #tf2 is second generation of the transform library that keeps track of multiple coordinate frames over time 
import geometry_msgs.msg #
import turtlesim.msg # turtle simulator
from nav_msgs.msg import Odometry


class PixhawkOrientation:
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    def handle_imu_pose(self, msg, imudata):
        coordinate_frame_broadcaster = tf2_ros.TransformBroadcaster()
        transform_pixhawk_to_imu = geometry_msgs.msg.TransformStamped() #expresses transform from coordinate frame pixhawk (header) to coordinate frame imu(child)

        transform_pixhawk_to_imu.header.stamp = rospy.Time.now()
        transform_pixhawk_to_imu.header.frame_id = "pixhawk" #header coordinate frame is called pixhawk
        transform_pixhawk_to_imu.child_frame_id = imudata #child is called imudata
        transform_pixhawk_to_imu.transform.translation.x = msg.x
        transform_pixhawk_to_imu.transform.translation.y = msg.y
        transform_pixhawk_to_imu.transform.translation.z = 0.0
        
        quaternion_to_euler = tf.transformations.euler_from_quaternion(self.roll, self.pitch, self.yaw)

        transform_pixhawk_to_imu.transform.rotation.x = quaternion_to_euler[0]
        transform_pixhawk_to_imu.transform.rotation.y = quaternion_to_euler[1]
        transform_pixhawk_to_imu.transform.rotation.z = quaternion_to_euler[2]
        transform_pixhawk_to_imu.transform.rotation.w = quaternion_to_euler[3]

        coordinate_frame_broadcaster.sendTransform


    def __init__(self):
        rospy.init_node('pixhawk_imu_broadcaster')
        imudata = rospy.get_param('~imu')
        rospy.Subscriber('/%s/pose' % imudata,
                        turtlesim.msg.Pose, #turtle simulator with pose messages
                        self.handle_imu_pose, #function
                        imudata)
        rospy.spin()
        
        

if __name__ == '__main__':
    pixhawkOrientation = PixhawkOrientation()

   
