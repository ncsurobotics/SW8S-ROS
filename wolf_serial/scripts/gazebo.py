#!/usr/bin/env python
import math
import rospy
import mavros
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Float64, Float32MultiArray
from mavros_msgs.msg import OverrideRCIn

# takes a value and remaps it from one range into another
def value_map(value, istart, istop, ostart, ostop):
    val = ostart + (ostop - ostart) * ((value - istart) / (istop - istart))
    if val >= ostop:
        return ostop
    if val <= ostart:
        return ostart

    return val


class Gazebo:
    pitch_rate = 0.0
    roll_rate = 0.0
    vertical_rate = 0.0
    yaw_rate = 0.0
    forward_rate = 0.0
    strafe_rate = 0.0

    current_yaw = 0.0
    current_depth = 0.0

    # transforms and their broadcasters
    coordinate_frame_broadcaster = None

    # move into class vars
    def vel_callback(self, data):
        self.pitch_rate = data.angular.y
        self.roll_rate = data.angular.x
        self.vertical_rate = data.linear.z
        self.yaw_rate = data.angular.z
        self.forward_rate = data.linear.x
        self.strafe_rate = data.linear.y

    # read the raw depth sensor data and publish it to the rest of our nodes
    def depth_callback(self, data):
        self.current_depth = data.data

    # read the raw orientation sensor data and publish it to the rest of our nodes
    # (THIS NEEDS TO BE CHANGED, CURRENTLY USES MAGNETIC SENSOR AND NOT IMU BECAUSE OF BROKEN SIMULATOR)
    def imu_callback(self, data):
        self.current_yaw = data.data

    # updates the hulls position in TF2
    def update_transform(self):
        hull_transform = TransformStamped()
        hull_transform.header.stamp = rospy.Time.now()
        hull_transform.header.frame_id = "map"
        hull_transform.child_frame_id = "odom"
        hull_transform.transform.translation.x = 0.0
        hull_transform.transform.translation.y = 0.0
        hull_transform.transform.translation.z = self.current_depth
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, math.radians(self.current_yaw))
        hull_transform.transform.rotation.x = q[0]
        hull_transform.transform.rotation.y = q[1]
        hull_transform.transform.rotation.z = q[2]
        hull_transform.transform.rotation.w = q[3]

        self.coordinate_frame_broadcaster.sendTransform(hull_transform)

    def __init__(self):
        # init ROS
        rospy.init_node('seawolf_gazebo_controller', anonymous=False)
        rate = rospy.Rate(20)
        mavros.set_namespace()

        # establish publishers for sensor data, and thruster controls
        rc = Float32MultiArray()
        rc.data.append(0.0);
        rc.data.append(0.0);
        rc.data.append(0.0);
        rc.data.append(0.0);
        rc.data.append(0.0);
        rc.data.append(0.0);

        self.override_pub = rospy.Publisher("wolf_RC_output", Float32MultiArray, queue_size=10)

        # subscribe to our target movement values as well as our raw sensor data
        rospy.Subscriber("cmd_vel", Twist, self.vel_callback)
        rospy.Subscriber("wolf_gazebo/global_alt", Float64, self.depth_callback)
        rospy.Subscriber("wolf_gazebo/compass_hdg", Float64, self.imu_callback)

        # establish coordinate frame and its broadcaster
        self.coordinate_frame_broadcaster = tf2_ros.TransformBroadcaster()
        self.update_transform()

        while not rospy.is_shutdown():
            # tells the thrusters to move to target rates, this is where movement actually occurs
            rc.data[0] = self.pitch_rate
            rc.data[1] = self.roll_rate
            rc.data[2] = self.vertical_rate
            rc.data[3] = self.yaw_rate
            rc.data[4] = self.forward_rate
            rc.data[5] = self.strafe_rate

            self.override_pub.publish(rc)
            self.update_transform()
            rate.sleep()


if __name__ == '__main__':
    gazebo = Gazebo()
