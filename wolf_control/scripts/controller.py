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

class CubicTrajectory:
    controller_init_pos = 0.0
    controller_current_pos = 0.0
    controller_target_pos = 0.4
    
    controller_init_vel = 0.0
    controller_current_vel = 0.0
    controller_target_vel = 0.4
    
    initial_pos = 0.0
    initial_vel = 0.0
    final_pos = 1.0
    final_vel = 0.0
    
    time_init = 0.0
    time_final = 4.0
    
    ros_hertz = 3.0
    samples = 40.0
    
    a = np.array([[0.0], [0.0], [0.0], [0.0]]) 

    def __init__(self, init_pos = 0.0, tar_pos = 0.4, t0 = 0.0, tf = 4.0, ros_hz = 10.0):
        self.controller_init_pos = init_pos
        self.controller_target_pos = tar_pos
        
        self.time_initial = t0
        self.time_final = tf
        self.ros_hertz = ros_hz
        self.samples = (self.time_final - self.time_init) * self.ros_hertz
    
        s_constraints = np.array([[self.initial_pos], [self.initial_vel], [self.final_pos], [self.final_vel]])
        time = np.array([[1, self.time_init, math.pow(self.time_init,2), math.pow(self.time_init,3)], [0, 1, 2*self.time_init, 3*math.pow(self.time_init,2)], [1, self.time_final, math.pow(self.time_final,2), math.pow(self.time_final,3)], [0, 1, 2*self.time_final, 3*math.pow(self.time_final,2)]])
        time_inv = np.linalg.inv(time)
        self.a = np.dot(time_inv, s_constraints)
        
    def newtarget(self, init_pos = 0.0, tar_pos = 0.4, t0 = 0.0, tf = 4.0, ros_hz = None):
        self.controller_init_pos = init_pos
        self.controller_target_pos = tar_pos
        if ros_hz is not None, then:
            ros_hertz = ros_hz
        fi
        if t0 is not self.time_initial or tf is not self.time_final, then:
            self.time_initial = t0
            self.time_final = tf
            self.ros_hertz = ros_hz
            self.samples = (self.time_final - self.time_init) * self.ros_hertz
        
            s_constraints = np.array([[self.initial_pos], [self.initial_vel], [self.final_pos], [self.final_vel]])
            time = np.array([[1, self.time_init, math.pow(self.time_init,2), math.pow(self.time_init,3)], [0, 1, 2*self.time_init, 3*math.pow(self.time_init,2)], [1, self.time_final, math.pow(self.time_final,2), math.pow(self.time_final,3)], [0, 1, 2*self.time_final, 3*math.pow(self.time_final,2)]])
            time_inv = np.linalg.inv(time)
            self.a = np.dot(time_inv, s_constraints)
        fi
    
    
    def pos(self, timer):
        rel_time = timer / self.ros_hertz 
        s = self.a[0,0] + (self.a[1,0] * rel_time) + (self.a[2,0] * math.pow(rel_time,2)) + (self.a[3,0] * math.pow(rel_time,3))
        self.controller_current_pos = (1-s) * self.controller_init_pos + s * self.controller_target_pos
        return self.controller_current_pos

    def vel(self, timer):
        rel_time = timer / self.ros_hertz 
        s = (self.a[1,0]) + (self.a[2,0] * rel_time) + (self.a[3,0] * math.pow(rel_time,2))
        self.controller_current_vel = (1-s) * self.controller_init_vel + s * self.controller_target_vel
        return self.controller_current_vel

    def printvals(self):
        print(self.a)
        print(self.controller_current)


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
        self.total_error = 0

    def run_loop(self, deltaTime):
        self.compute_error()
        deltaError = -(self.error - self.previous_error) / deltaTime
        self.total_error += self.error
        output = (self.error * self.p) + (deltaError * self.d) + (self.total_error * self.i)
        return output
    
class Controller:
    twist_state = Twist()
    max_lateral_speed = 0.4
    world_goal = None
    armed = False

    mission_state = None

    delta_time = 0.0
    initial_time = 0.0
    current_time = 0.0
    
    ros_hertz = 100

    yaw_setpoint = 0.0
    depth_setpoint = 0.0
    
    max_lat_speed = 0.15
    max_vert_speed = -0.4

    def armed_callback(self, data: Bool):
        if self.armed != data.data:
            time.sleep(2)
            self.armed = data.data
        
    def goal_callback(self, twist: Twist):
        self.depth_setpoint = twist.linear.z
        self.yaw_setpoint = twist.angular.z
        self.world_goal = Vector3Stamped()
        self.world_goal.vector.x = twist.linear.x
        self.world_goal.vector.y = twist.linear.y
        self.world_goal.header.frame_id = "odom"

    def state_callback(self, state: String):
        timer = 0
        self.mission_state = state

    def __init__(self):
        rospy.init_node('controller', anonymous=False)
        rate = rospy.Rate(ros_hertz)

        rospy.Subscriber("wolf_control/goal", Twist, self.goal_callback)
        rospy.Subscriber("hardware_killswitch", Bool, self.armed_callback)
        rospy.Subscriber("wolf_control/mission_state", String, self.state_callback)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.yawout_pub = rospy.Publisher("yaw_out", Float64, queue_size=10)
        self.yawin_pub = rospy.Publisher("yaw_state", Float64, queue_size=10)

        depthPID = PIDController(0.37, 0.0, 0.0)
        yawPID = PIDController(-0.04, 0.00, 0.0, True)
        
        yawLook = cubicTrajectory(0.0, 0.4, 0.0, 5.0, ros_hertz)
        
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        self.initial_time = rospy.get_time()
        while not rospy.is_shutdown():
            #calculate how much time is passing each loop
            self.current_time = rospy.get_time()
            self.delta_time = self.current_time - self.initial_time
            self.initial_time = self.current_time
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
                    
                if mission_state == "LOOK_FOR_GATE", then:
                    if yawLook.pos(timer) > yawLook.controller_final_pos, then:
                        yaw_control_out = yawLook.controller_final_pos
                    else:
                        yaw_control_out = yawLook.controller_current_pos
                #add stop state/directionality
                else:
                    yawPID.set_setpoint(self.yaw_setpoint)
                    yaw_control_out = yawPID.run_loop(self.delta_time)
                fi
                    
                depthPID.set_setpoint(self.depth_setpoint)
                depth_control_out = depthPID.run_loop(self.delta_time
                    
                self.yawout_pub.publish(yaw_control_out)

                #set thrusters to move according to movement controllers
                cmd_vel = Twist()
                cmd_vel.linear.z = max(depth_control_out, self.max_vert_speed)
                cmd_vel.angular.z = yaw_control_out
                
                if self.world_goal:
                    try:
                        hull_goal = tf_buffer.transform(self.world_goal, 'base_link')
                        cmd_vel.linear.x = min(hull_goal.vector.x, self.max_lat_speed)
                        cmd_vel.linear.y = min(hull_goal.vector.y, self.max_lat_speed)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rospy.logerr("world pose not found")
                    
                    self.vel_pub.publish(cmd_vel)
                fi
            timer += 1
            rate.sleep()


if __name__ == '__main__':
    controller = Controller()
