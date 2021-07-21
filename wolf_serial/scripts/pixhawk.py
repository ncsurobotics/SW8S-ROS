#!/usr/bin/env python
import math
import rospy
import mavros
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped, Twist, PoseWithCovarianceStamped
from std_msgs.msg import Float64, Bool
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import *
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

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
    
    arm_service = None
    mode_serivce = None
    armed = False

    # convert normalized target values into RC rates
    def vel_callback(self, data: Twist):
        self.pitch_rate = value_map(data.angular.y, -1.0, 1.0, 1000, 2000)
        self.roll_rate = value_map(data.angular.x, -1.0, 1.0, 1000, 2000)
        self.vertical_rate = value_map(data.linear.z, -1.0, 1.0, 1000, 2000)
        self.yaw_rate = value_map(data.angular.z, -1.0, 1.0, 1000, 2000)
        self.forward_rate = value_map(data.linear.x, -1.0, 1.0, 1000, 2000)
        self.strafe_rate = value_map(data.linear.y, -1.0, 1.0, 1000, 2000)

    # read the raw depth sensor data and publish it to the rest of our nodes
    def depth_callback(self, data: Float64):
        self.current_depth = data.data

        # check how much time has passed since last update
        self.delta_time = rospy.get_time() - self.initial_time
        self.initial_time = rospy.get_time()

        depth_pose = PoseWithCovarianceStamped()
        depth_pose.header.stamp = rospy.Time.now()
        depth_pose.header.frame_id = "odom"
        depth_pose.pose.pose.position.z = data.data
        identity = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        depth_pose.pose.covariance = [i * 0.05 for i in identity]
        self.depth_pose_pub.publish(depth_pose)
    
    def killswitch_callback(self, data: Bool):
        if self.armed != data.data:
            rospy.logerr(data.data)
            self.armed = data.data
            self.set_mode(0, 'ALT_HOLD')
            self.arm(data.data)
        

    def arm(self, should_arm: bool):
        if self.arm_service is not None: 
            result = self.arm_service(should_arm)
            if result.success == False:
                self.arm(should_arm)
    
    def set_mode(self, base_mode: int, custom_mode: str):
        if self.mode_service is not None:
            result = self.mode_service(base_mode, custom_mode)
            if result.mode_sent == False:
                self.set_mode(base_mode, custom_mode)

    def __init__(self):
        # init ROS
        rospy.init_node('pixhawk', anonymous=False)
        rate = rospy.Rate(20)
        mavros.set_namespace()
        
        #Get Mavros Services
        rospy.wait_for_service("mavros/cmd/arming")
        rospy.wait_for_service("mavros/set_mode")
        self.arm_service = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.mode_service = rospy.ServiceProxy("mavros/set_mode", SetMode)
         
        self.set_mode(0, 'ALT_HOLD')
        #self.arm(True)
        
        # time setup
        self.initial_time = rospy.get_time()

        # establish publishers for sensor data, and thruster controls
        rc = OverrideRCIn()
        self.override_pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10)
        self.depth_pose_pub = rospy.Publisher("wolf_serial/depth_pose", PoseWithCovarianceStamped, queue_size=10)

        # subscribe to our target movement values as well as our raw sensor data
        rospy.Subscriber("cmd_vel", Twist, self.vel_callback)
        rospy.Subscriber("mavros/global_position/rel_alt", Float64, self.depth_callback)
        rospy.Subscriber("hardware_killswitch", Bool, self.killswitch_callback)

        # establish coordinate frame and its broadcaster
        self.coordinate_frame_broadcaster = tf2_ros.TransformBroadcaster()
#        self.update_transform()

        while not rospy.is_shutdown():
            # tells the thrusters to move to target rates, this is where movement actually occurs
            rc.channels[0] = int(self.pitch_rate)
            rc.channels[1] = int(self.roll_rate)
            rc.channels[2] = int(self.vertical_rate)
            rc.channels[3] = int(self.yaw_rate)
            rc.channels[4] = int(self.forward_rate)
            rc.channels[5] = int(self.strafe_rate)

            '''
            # if no data is coming in, kill thrusters
            if self.delta_time > 0.1:
                rc.channels[0] = 1500
                rc.channels[1] = 1500
                rc.channels[2] = 1500
                rc.channels[3] = 1500
                rc.channels[4] = 1500
                rc.channels[5] = 1500
                rospy.logerr("WATCHDOG TIMER TRIGGERED: SENSOR DATA IS TOO SLOW")
'''
            self.override_pub.publish(rc)
#            self.update_transform()
            rate.sleep()


if __name__ == '__main__':
    pixhawk = Pixhawk()
