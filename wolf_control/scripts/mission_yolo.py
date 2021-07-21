#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String
from enum import Enum
import tf2_ros
import math
import numpy as np

class mission_states(Enum):
    STOP = -1
    SUBMERGE = 0
    MOVE_TO_GATE = 1
    MOVE_THROUGH_GATE = 2

def checkTolerance(current, wanted):
    tolerance = 0.3
    return (current < (wanted + tolerance)) and (current > (wanted - tolerance))
        
def mission():
    rospy.init_node('mission_controller', anonymous=True)
    state = mission_states.SUBMERGE
    goal_pub = rospy.Publisher('wolf_control/goal', Twist, queue_size=10)
    state_pub = rospy.Publisher('wolf_control/mission_state', String, queue_size=10)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10) # 10hz

    timer = 0
    saved_goal = None
    right_gate_queue = []
    right_angle_queue = []
    left_gate_queue = []

    #hyper params
    submerge_depth = -2.35
    dead_reckon_duration = 100
    queue_depth = 10
    sigma_tolerance = 5
    should_turn = False
    should_discard_stale = True

    last_world_gate = None
    last_base_gate = None
    no_gate_count = 0
    good_gate_count = 0 

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
                goal.angular.z = 3.1415 + odom.transform.rotation.z
                goal_pub.publish(goal)
                if (abs(odom.transform.translation.z - submerge_depth)) < 0.3:
                    state = mission_states.MOVE_TO_GATE  
                    timer = 0
                    saved_goal = None        
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
                    time_since_frame = abs(world_right_gate.header.stamp - rospy.Time.now())
                    #maintain a queue of the last 5 gate transforms
                    if len(right_gate_queue) > queue_depth:
                        stddev = np.std(right_gate_queue)
                        avg = np.mean(right_gate_queue)
                        lower = avg - (stddev * sigma_tolerance)
                        upper = avg + (stddev * sigma_tolerance)
                        right_gate_queue.pop(0) # pop something off the queue to make room for the next datapoint
                        right_angle_queue.pop(0)

                        # if we're within the normal data range this is a stable point
                        if world_right_gate.transform.translation.x < upper and world_right_gate.transform.translation.x > lower:
                            right_stable = True
                    else:
                        right_stable = True

                    right_gate_queue.append(world_right_gate.transform.translation.x)
                    angle_to_gate = math.atan2(base_right_gate.transform.translation.y, base_right_gate.transform.translation.x)
                    right_angle_queue.append(angle_to_gate)

                    #stale frames don't count
                    if should_discard_stale and time_since_frame.secs > 2:
                        right_stable = False

                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                    pass
                #left gate is too flakey, none of this is used right now
                try:
                    world_left_gate = tf_buffer.lookup_transform("odom", "left_gate", rospy.Time(0))
                    base_left_gate = tf_buffer.lookup_transform("base_link", "left_gate", rospy.Time(0))
                    #maintain a queue of the last 5 gate transforms
                    if len(left_gate_queue) > queue_depth:
                        stddev = np.std(left_gate_queue)
                        avg = np.mean(left_gate_queue)
                        lower = avg - (stddev * sigma_tolerance)
                        upper = avg + (stddev * sigma_tolerance)
                        left_gate_queue.pop(0) # pop something off the queue to make room for the next datapoint

                        # if we're within the normal data range this is a stable point
                        if world_left_gate.transform.translation.x < upper and world_left_gate.transform.translation.x > lower:
                            left_stable = True
                    else:
                        left_stable = True

                    left_gate_queue.append(world_left_gate.transform.translation.x)
                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                    pass
                
                # move towards the stable point
                if right_stable:
                    world_gate_vector = world_right_gate
                    base_gate_vector = base_right_gate
                    no_gate_count = 0
                    good_gate_count += 1
                elif len(right_gate_queue) > queue_depth:
                    good_gate_count = 0
                    no_gate_count += 1
                    rospy.logwarn("missed one")

                # record our current heading in case next data point isnt stable
                last_world_gate = world_gate_vector
                last_base_gate = base_gate_vector
                
                if base_gate_vector is not None and world_gate_vector is not None:
                    angle_to_gate = math.atan2(base_gate_vector.transform.translation.y, base_gate_vector.transform.translation.x)
                    goal = Twist()
                    goal.linear.x = world_gate_vector.transform.translation.x
                    goal.linear.y = world_gate_vector.transform.translation.y
                    goal.linear.z = submerge_depth
                    if should_turn and good_gate_count > queue_depth and len(right_gate_queue) > queue_depth:
                        goal.angular.z = odom.transform.rotation.z + np.mean(right_angle_queue) + 3.1415
                        pass
                    goal_pub.publish(goal)
                if no_gate_count > 6:
                    rospy.logwarn("missed too many, dead reckoning")
                    saved_goal = goal
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
