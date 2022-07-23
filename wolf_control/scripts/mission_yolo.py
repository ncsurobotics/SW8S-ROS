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
    LOOK_FOR_GATE = 1
    MOVE_TO_GATE = 2
    MOVE_THROUGH_GATE = 3

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
    listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10) # 10hz

    timer = 0
    saved_goal = None
    right_gate_queue = []
    right_angle_queue = []
    left_gate_queue = []

    #hyper params
    submerge_depth = -2.25
    dead_reckon_duration = 10
    queue_depth = 4
    sigma_tolerance = 5
    should_turn = True
    should_discard_stale = True

    last_world_gate = None
    last_base_gate = None
    no_left_count = 0
    no_right_count = 0
    left_count = 0
    right_count = 0
    no_gate_count = 0
    good_gate_count = 0 
    search_angle = 0

    while not rospy.is_shutdown():
        try:
            odom: TransformStamped = tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
            if state == mission_states.STOP:
                goal = Twist()
                goal.linear.z = -0.2
                goal_pub.publish(goal)
            elif state == mission_states.WAIT_FOR_ARM:
                if arm == False:
                    timer = 0
                else:
                    rospy.logerr("counting")
                if timer > 4:
                    state = mission_states.SUBMERGE
                    saved_goal = odom
                    timer = 0
            elif state == mission_states.SUBMERGE:
                goal = Twist()
                goal.linear.z = submerge_depth
                goal.angular.z = 3.1415 + saved_goal.transform.rotation.z - 0.261799388
                goal_pub.publish(goal)
                if (abs(odom.transform.translation.z - submerge_depth)) < 0.3:
                    state = mission_states.MOVE_TO_GATE
                    timer = 0
                    saved_goal = None        
                    search_angle = odom.transform.rotation.z
            elif state == mission_states.LOOK_FOR_GATE:
                world_right_gate = None
                base_right_gate = None

                world_left_gate = None
                base_left_gate = None
                                
                world_gate_vector = None
                base_gate_vector = None

                try:
                    world_right_gate = tf_buffer.lookup_transform("odom", "gate", rospy.Time(0))
                    base_right_gate = tf_buffer.lookup_transform("base_link", "gate", rospy.Time(0))
                    if world_right_gate is not None:
                        if (world_right_gate.header.stamp - rospy.Time(0).now()).to_sec() > 1:
                            world_right_gate = None
                            base_right_gate = None
                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                    #rospy.logerr("no right gate")
                    pass

                try:
                    world_left_gate = tf_buffer.lookup_transform("odom", "left_gate", rospy.Time(0))
                    base_left_gate = tf_buffer.lookup_transform("base_link", "left_gate", rospy.Time(0))
                    if world_left_gate is not None:
                        if (world_left_gate.header.stamp - rospy.Time(0).now()).to_sec() > 1:
                            world_left_gate = None
                            base_left_gate = None
                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                    #rospy.logerr("no left gate")
                    pass
                
                if world_right_gate is not None:
                    #if no_left_count > 5:
                    world_gate_vector = world_right_gate
                    base_gate_vector = base_right_gate
                    no_right_count = 0
                    right_count += 1
                else:
                    no_right_count += 1
                    right_count = 0
                if world_left_gate is not None:
                    world_gate_vector = world_left_gate
                    base_gate_vector = base_left_gate
                    no_left_count = 0
                    left_count += 1
                else:
                    no_left_count += 1
                    left_count = 0

                if left_count > 1 and right_count > 1:
                    state = mission_states.MOVE_TO_GATE  
                    timer = 0
                    saved_goal = None        
                elif timer > 100:
                    search_angle = ((search_angle + 0.5 + 3.14) % 6.38) - 3.14
                    goal = Twist()
                    goal.linear.x = 0
                    goal.linear.y = 0
                    goal.linear.z = submerge_depth
                    goal.angular.z = search_angle
                    goal_pub.publish(goal)
                    timer = 0
            elif state == mission_states.MOVE_TO_GATE:
                # find which gate vector is more stable
                world_right_gate = None
                base_right_gate = None
                right_stable = False

                world_left_gate = None
                base_left_gate = None
                left_stable = False

                world_gate_vector = last_world_gate
                base_gate_vector = last_base_gate
                try:
                    world_right_gate = tf_buffer.lookup_transform("odom", "gate", rospy.Time(0))
                    base_right_gate = tf_buffer.lookup_transform("base_link", "gate", rospy.Time(0))
                    if world_right_gate is not None:
                        if (world_right_gate.header.stamp - rospy.Time(0).now()).to_sec() > 1:
                            world_right_gate = None
                            base_right_gate = None
                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                    rospy.logerr("no right gate")
                try:
                    world_left_gate = tf_buffer.lookup_transform("odom", "left_gate", rospy.Time(0))
                    base_left_gate = tf_buffer.lookup_transform("base_link", "left_gate", rospy.Time(0))
                    if world_left_gate is not None:
                        if (world_left_gate.header.stamp - rospy.Time(0).now()).to_sec() > 1:
                            world_left_gate = None
                            base_left_gate = None
                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                    rospy.logerr("no left gate")
                
                # move towards the stable point
                if world_right_gate is not None:
                    #if no_left_count > 5:
                    world_gate_vector = world_right_gate
                    base_gate_vector = base_right_gate
                    no_right_count = 0
                    right_count += 1
                else:
                    no_right_count += 1
                if world_left_gate is not None:
                    world_gate_vector = world_left_gate
                    base_gate_vector = base_left_gate
                    no_left_count = 0
                    left_count += 1
                else:
                    no_left_count += 1

                if base_gate_vector is not None and world_gate_vector is not None:
                    angle_to_gate = math.atan2(base_gate_vector.transform.translation.y, base_gate_vector.transform.translation.x)
                    rospy.logerr("should be good")
                    goal = Twist()
                    goal.linear.x = world_gate_vector.transform.translation.x
                    goal.linear.y = world_gate_vector.transform.translation.y
                    goal.linear.z = submerge_depth
                    if should_turn:
                        goal.angular.z = odom.transform.rotation.z + angle_to_gate + 3.1415
                        pass
                    saved_goal = goal
                    goal_pub.publish(goal)
                    rospy.logerr("left: " + str(no_left_count) + " right: " + str(no_right_count))
                if no_left_count > 0 and no_right_count > 0:
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
