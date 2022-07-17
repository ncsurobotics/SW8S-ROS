#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String, Bool
from enum import Enum
import tf2_ros
import math
import numpy as np


class mission_states(Enum):
    WAIT_FOR_ARM = -2 
    STOP = -1
    SUBMERGE = 0
    MOVE_TO_GATE = 1
    MOVE_THROUGH_GATE = 2

arm = True
state = mission_states.WAIT_FOR_ARM

def checkTolerance(current, wanted):
    tolerance = 0.3
    return (current < (wanted + tolerance)) and (current > (wanted - tolerance))

def setArm(data):
    global arm
    global state
    if arm != data.data:
        arm = data.data
        state = mission_states.WAIT_FOR_ARM

def mission():
    global arm
    global state
    rospy.init_node('mission_controller', anonymous=True)
    goal_pub = rospy.Publisher('wolf_control/goal', Twist, queue_size=10)
    state_pub = rospy.Publisher('wolf_control/mission_state', String, queue_size=10)
    rospy.Subscriber("hardware_killswitch", Bool, setArm)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)     #what does this do?
    rate = rospy.Rate(10) # 10hz

    timer = 0
    saved_goal = None
    right_gate_queue = []
    right_angle_queue = []
    left_gate_queue = []

    #hyper params
    submerge_depth = -2.45
    dead_reckon_duration = 30
    queue_depth = 4
    depth_tolerance = 0.3
    sigma_tolerance = 5
    should_turn = True
    should_discard_stale = True

    last_world_gate = None      #these are not updated
    last_base_gate = None       #these are not updated
    no_left_count = 0
    no_right_count = 0
    left_count = 0
    right_count = 0
    no_gate_count = 0
    good_gate_count = 0 

    while not rospy.is_shutdown():
        try:
            odom: TransformStamped = tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0)) # assuming this is an anonymous function that also runs at the top of each loop is it necessary
            if state == mission_states.STOP:
                goal = Twist()
                goal.linear.z = submerge_depth
                goal_pub.publish(goal)
            elif state == mission_states.WAIT_FOR_ARM:
                if arm == False:
                    timer = 0
                else:
                    rospy.logerr("counting")
                if timer > queue_depth:
                    state = mission_states.SUBMERGE
                    saved_goal = odom   #this odom call seems redundant could use TransformStamped?
                    timer = 0
            elif state == mission_states.SUBMERGE:
                goal = Twist()
                goal.linear.z = submerge_depth
                goal.angular.z = 3.1415 + saved_goal.transform.rotation.z  #Why does it need to rotate 3.1415?
                goal_pub.publish(goal)
                if (abs(odom.transform.translation.z - submerge_depth)) < depth_tolerance: #this odom call seems redundant could use TransformStamped?
                    state = mission_states.MOVE_TO_GATE  
                    timer = 0
                    saved_goal = None        
            elif state == mission_states.MOVE_TO_GATE:
                # find which gate vector is more stable
                world_right_gate = None
                base_right_gate = None
                right_stable = False    #unused

                world_left_gate = None
                base_left_gate = None
                left_stable = False     #unused

                world_gate_vector = last_world_gate #originally always set to None
                base_gate_vector = last_base_gate   #originally always set to None
                try:
                    world_right_gate = tf_buffer.lookup_transform("odom", "gate", rospy.Time(0))
                    base_right_gate = tf_buffer.lookup_transform("base_link", "gate", rospy.Time(0))    #could this be derived from TransformStamped and world_right_gate
                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                    rospy.logerr("no right gate")
                try:
                    world_left_gate = tf_buffer.lookup_transform("odom", "left_gate", rospy.Time(0))    
                    base_left_gate = tf_buffer.lookup_transform("base_link", "left_gate", rospy.Time(0))    #could this be derived from TransformStamped and world_left_gate
                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                    rospy.logerr("no left gate")
                
                # move towards the stable point
#                if world_right_gate is not None:
#                    if no_left_count > 5:
#                        world_gate_vector = world_right_gate
#                        base_gate_vector = base_right_gate
#                    no_right_count = 0
#                    right_count += 1
#                else:
#                    no_right_count += 1
#                if world_left_gate is not None:
#                   world_gate_vector = world_left_gate
#                   base_gate_vector = base_left_gate
#                    no_left_count = 0
#                    left_count += 1
#                else:
#                    no_left_count += 1
                   
                if world_left_gate is not None:
                    if no_left_count <= no_right_count or left_count >= right_count:
                        world_gate_vector = world_left_gate
                        base_gate_vector = base_left_gate
                    no_left_count = 0
                    left_count += 1
                else:
                    no_left_count += 1
                    if no_left_count > 2
                        left_count = 0
                if world_right_gate is not None:
                    if world_gate_vector == last_world_gate:
                        world_gate_vector = right_world_gate
                        base_gate_vector = right_base_gate
                    no_right_count = 0
                    right_count += 1
                else:
                    no_right_count += 1
                    if no_right_count > 2
                        left_count = 0

                if base_gate_vector is not None and world_gate_vector is not None:
                    angle_to_gate = math.atan2(base_gate_vector.transform.translation.y, base_gate_vector.transform.translation.x)
                    rospy.logerr("should be good")
                    goal = Twist()
                    goal.linear.x = world_gate_vector.transform.translation.x
                    goal.linear.y = world_gate_vector.transform.translation.y
                    goal.linear.z = submerge_depth
                    if should_turn:
                        goal.angular.z = odom.transform.rotation.z + angle_to_gate + 3.1415
                        pass    #why pass?
                    saved_goal = goal
                    goal_pub.publish(goal)
                    last_base_gate = base_gate_vector
                    last_world_gate = world_gate_vector
                if no_left_count > 3 and no_right_count > 3:
                    rospy.logwarn("missed too many, dead reckoning")
                    state = mission_states.MOVE_THROUGH_GATE
                    timer = 0
            elif state == mission_states.MOVE_THROUGH_GATE:
                goal_pub.publish(saved_goal)
                if timer > dead_reckon_duration:
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
