#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String
from enum import Enum
import tf2_ros
import math

class mission_states(Enum):
    STOP = -1
    SUBMERGE = 0
    MOVE_TO_GATE = 1
    MOVE_THROUGH_GATE = 2

def checkTolerance(current, wanted):
    tolerance = 0.1
    return current < wanted + tolerance and current > wanted - tolerance
        
def mission():
    rospy.init_node('mission_controller', anonymous=True)
    state = mission_states.SUBMERGE
    goal_pub = rospy.Publisher('wolf_control/goal', Twist, queue_size=10)
    state_pub = rospy.Publisher('wolf_control/mission_state', String, queue_size=10)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10) # 10hz

    submerge_depth = -1.5
    timer = 0
    saved_goal = None
    while not rospy.is_shutdown():
        try:
            odom: TransformStamped = tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
            if state == mission_states.STOP:
                goal = Twist()
                goal.linear.z = submerge_depth
                goal_pub.publish(goal)
            if state == mission_states.SUBMERGE:
                goal = Twist()
                goal.linear.z = submerge_depth
                goal.angular.z = odom.transform.rotation.z
                goal_pub.publish(goal)
                if checkTolerance(odom.transform.translation.z, submerge_depth):
                    state = mission_states.MOVE_TO_GATE  
                    timer = 0
                    saved_goal = None        
            elif state == mission_states.MOVE_TO_GATE:
                gate_vector: TransformStamped = tf_buffer.lookup_transform("odom", "gate", rospy.Time(0))
                goal = Twist()
                goal.linear.x = gate_vector.transform.translation.x * 0.1
                goal.linear.y = gate_vector.transform.translation.y * 0.1
                goal.linear.z = submerge_depth
                goal_pub.publish(goal)
                if timer > 80:
                    saved_goal = goal
                    state = mission_states.MOVE_THROUGH_GATE
                    timer = 0
            elif state == mission_states.MOVE_THROUGH_GATE:
                goal_pub.publish(saved_goal)
                if timer > 170:
                    timer = 0
                    saved_goal = None
                    state = mission_states.STOP
            timer += 1
            state_pub.publish(state.name)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("mission_code: error finding frame")
        rate.sleep()
if __name__ == '__main__':
    try:
        mission()
    except rospy.ROSInterruptException:
        pass
