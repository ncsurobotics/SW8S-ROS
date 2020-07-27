#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import Vector3Stamped
from std_msgs.msg import Float64
import tf2_ros

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
    world_goal = None

    def twist_callback(self, twist):
        self.depth_set_pub.publish(twist.linear.z)
        self.yaw_set_pub.publish(twist.angular.z)
        self.world_goal = Vector3Stamped()
        self.world_goal.vector.x = twist.linear.x
        self.world_goal.vector.y = twist.linear.y
        self.world_goal.header.frame_id = "world"

    def depth_control_callback(self, controller_output):
        self.depth_control_out = controller_output.data

    def yaw_control_callback(self, controller_output):
        self.yaw_control_out = controller_output.data

    def __init__(self):
        rospy.init_node('controller', anonymous=False)
        rate = rospy.Rate(20)

        rospy.Subscriber("wolf_twist_setpoint", Twist, self.twist_callback)
        self.yaw_set_pub = rospy.Publisher("wolf_control/yaw_setpoint", Float64, queue_size=10)
        self.depth_set_pub = rospy.Publisher("wolf_control/depth_setpoint", Float64, queue_size=10)

        rospy.Subscriber("wolf_control/yaw_output", Float64, self.yaw_control_callback)
        rospy.Subscriber("wolf_control/depth_output", Float64, self.depth_control_callback)
        self.twist_pub = rospy.Publisher("wolf_twist", Twist, queue_size=10)

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        while not rospy.is_shutdown():
            twist_out = Twist()
            twist_out.linear.z = self.depth_control_out
            twist_out.angular.z = self.yaw_control_out

            if self.world_goal:
                try:
                    hull_goal = tf_buffer.transform(self.world_goal, 'wolf_hull')
                    twist_out.linear.x = hull_goal.vector.x
                    twist_out.linear.y = hull_goal.vector.y
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass

            self.twist_pub.publish(twist_out)
            rate.sleep()


if __name__ == '__main__':
    controller = Controller()
