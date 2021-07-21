#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import Vector3Stamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64, Bool
import tf2_ros
import tf_conversions
import time

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

class PIDController:
    state = 0.0
    setpoint = 0.0
    error = 0.0
    previous_error = 0.0
    total_error = 0.0

    p = 0.0
    i = 0.0
    d = 0.0

    is_angle = False

    def __init__(self, p, i, d, is_angle = False):
        self.p = p
        self.i = i
        self.d = d
        self.is_angle = is_angle

    def anglediff(self, a,b):
        distances = [b-a, b - ((2*math.pi) - a), ((2*math.pi) - b) - a, b - ((2*math.pi) + a), ((2*math.pi) + b) - a]
        absdist = [abs(number) for number in distances]
        index = min(range(len(absdist)), key=absdist.__getitem__)
        return distances[index]

    def compute_error(self):
        self.previous_error = self.error
        if self.is_angle:
            self.error = self.anglediff(self.setpoint, self.state)
        else:
            self.error = self.setpoint - self.state

    def set_state(self, newstate):
        self.state = newstate
    
    def set_setpoint(self, newset):
        self.setpoint = newset

    def run_loop(self, deltaTime):
        self.compute_error()
        deltaError = (self.error - self.previous_error) / deltaTime
        self.total_error += self.error
        output = (self.error * self.p) + (deltaError * self.d) + (self.total_error * self.i)
        return output
    
class Controller:
    twist_state = Twist()
    max_lateral_speed = 0.4
    world_goal = None
    armed = True

    delta_time = 0.0
    initial_time = 0.0

    yaw_setpoint = 0.0
    depth_setpoint = 0.0

    def armed_callback(self, data: Bool):
        if self.armed != data.data:
            rospy.logerr(data.data)
            self.armed = data.data
        
    def goal_callback(self, twist: Twist):
        self.depth_setpoint = twist.linear.z
        self.yaw_setpoint = twist.angular.z
        self.world_goal = Vector3Stamped()
        self.world_goal.vector.x = twist.linear.x
        self.world_goal.vector.y = twist.linear.y
        self.world_goal.header.frame_id = "odom"

    def __init__(self):
        rospy.init_node('controller', anonymous=False)
        rate = rospy.Rate(20)

        rospy.Subscriber("wolf_control/goal", Twist, self.goal_callback)
        rospy.Subscriber("hardware_killswitch", Bool, self.armed_callback)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.yawout_pub = rospy.Publisher("yaw_out", Float64, queue_size=10)
        self.yawin_pub = rospy.Publisher("yaw_state", Float64, queue_size=10)


        depthPID = PIDController(0.45, 0.0, 0.0)
        yawPID = PIDController(-0.04, 0.0, 0.0, True)

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        self.initial_time = rospy.get_time()
        while not rospy.is_shutdown():
            #calculate how much time is passing each loop
            self.delta_time = rospy.get_time() - self.initial_time
            self.initial_time = rospy.get_time()
            if self.armed:
                #get the current robot position and give the appropriate pieces to the PID controllers
                try:
                    #transform goal data into base_link space
                    odom = tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
                    rot = (odom.transform.rotation.x, odom.transform.rotation.y, odom.transform.rotation.z, odom.transform.rotation.w)

                    #give states to the PID controllers
                    yawPID.set_state(tf_conversions.transformations.euler_from_quaternion(rot)[2])
                    depthPID.set_state(odom.transform.translation.z)
                    self.yawin_pub.publish(tf_conversions.transformations.euler_from_quaternion(rot)[2])

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print("failed to get transform")
                    pass

                yawPID.set_setpoint(self.yaw_setpoint)
                depthPID.set_setpoint(self.depth_setpoint)
                
                yaw_control_out = yawPID.run_loop(self.delta_time)
                depth_control_out = depthPID.run_loop(self.delta_time)
                self.yawout_pub.publish(yaw_control_out)

                #set thrusters to move according to movement controllers
                cmd_vel = Twist()
                cmd_vel.linear.z = depth_control_out
                cmd_vel.angular.z = yaw_control_out
                
                if self.world_goal:
                    try:
                        hull_goal = tf_buffer.transform(self.world_goal, 'base_link')
                        cmd_vel.linear.x = hull_goal.vector.x
                        cmd_vel.linear.y = hull_goal.vector.y
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rospy.logerr("world pose not found")

                self.vel_pub.publish(cmd_vel)
            rate.sleep()


if __name__ == '__main__':
    controller = Controller()
