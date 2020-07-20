#!/usr/bin/env python
import math
import rospy
import mavros
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Float64
from mavros_msgs.msg import OverrideRCIn

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

    current_yaw = 0.0
    current_depth = 0.0
    # watchdog timer variables
    # (checks to make sure data is flowing into the controller or kills thrusters
    delta_time = 0.0
    initial_time = 0.0

    # transforms and their broadcasters
    coordinate_frame_broadcaster = None

    # convert normalized target values into RC rates
    def twist_callback(self, data):
        self.pitch_rate = value_map(data.angular.y, -1.0, 1.0, 1000, 2000)
        self.roll_rate = value_map(data.angular.x, -1.0, 1.0, 1000, 2000)
        self.vertical_rate = value_map(data.linear.z, -1.0, 1.0, 1000, 2000)
        self.yaw_rate = value_map(data.angular.z, -1.0, 1.0, 1000, 2000)
        self.forward_rate = value_map(data.linear.x, -1.0, 1.0, 1000, 2000)
        self.strafe_rate = value_map(data.linear.y, -1.0, 1.0, 1000, 2000)

    # read the raw depth sensor data and publish it to the rest of our nodes
    def depth_callback(self, data):
        self.current_depth = data.data
        self.depth_pub.publish(data.data)

        # check how much time has passed since last update
        self.delta_time = rospy.get_time() - self.initial_time
        self.initial_time = rospy.get_time()

        self.update_transform()

    # read the raw orientation sensor data and publish it to the rest of our nodes
    # (THIS NEEDS TO BE CHANGED, CURRENTLY USES MAGNETIC SENSOR AND NOT IMU BECAUSE OF BROKEN SIMULATOR)
    def imu_callback(self, data):
        self.current_yaw = data.data
        self.yaw_pub.publish(data.data)

        # check how much time has passed since last update
        self.delta_time = rospy.get_time() - self.initial_time
        self.initial_time = rospy.get_time()

        self.update_transform()

    # updates the hulls position in TF2
    def update_transform(self):
        hull_transform = TransformStamped()
        hull_transform.header.stamp = rospy.Time.now()
        hull_transform.header.frame_id = "world"
        hull_transform.child_frame_id = "wolf_hull"
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
        rospy.init_node('pixhawk', anonymous=False)
        rate = rospy.Rate(20)
        mavros.set_namespace()

        # time setup
        self.initial_time = rospy.get_time()

        # establish publishers for sensor data, and thruster controls
        rc = OverrideRCIn()
        self.override_pub = rospy.Publisher(mavros.get_topic("rc", "override"), OverrideRCIn, queue_size=10)
        self.yaw_pub = rospy.Publisher("wolf_yaw", Float64, queue_size=10)
        self.depth_pub = rospy.Publisher("wolf_depth", Float64, queue_size=10)

        # subscribe to our target movement values as well as our raw sensor data
        rospy.Subscriber("wolf_twist", Twist, self.twist_callback)
        rospy.Subscriber("mavros/global_position/rel_alt", Float64, self.depth_callback)
        rospy.Subscriber("mavros/global_position/compass_hdg", Float64, self.imu_callback)

        # establish coordinate frame and its broadcaster
        self.coordinate_frame_broadcaster = tf2_ros.TransformBroadcaster()
        self.update_transform()

        while not rospy.is_shutdown():
            # tells the thrusters to move to target rates, this is where movement actually occurs
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
