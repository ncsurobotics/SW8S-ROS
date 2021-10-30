#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String
from enum import Enum
import tf2_ros
import math
from sm import *


class STOP(State):
    def execute(args):
        goal = Twist()
        goal.linear.z = args["submerge_depth"]
        args["goal_pub"].publish(goal)


class SUBMERGE(State):
    def execute(args):
        goal = Twist()
        goal.linear.z = args["submerge_depth"]
        args["goal_pub"].publish(goal)
        if checkTolerance(args["odom"].transform.translation.z, args["submerge_depth"]):
            args["timer"] = 0
            args["saved_goal"] = None
            return (MOVE_TO_GATE,0)


class MOVE_TO_GATE(State):
    def execute(args):
        gate_vector: TransformStamped = args["tf_buffer"].lookup_transform(
            "odom", "gate", rospy.Time(0)
        )
        goal = Twist()
        goal.linear.x = gate_vector.transform.translation.x * 1.3
        goal.linear.y = gate_vector.transform.translation.y
        goal.linear.z = args["submerge_depth"]
        goal.angular.z = -math.atan2(
            gate_vector.transform.translation.y, gate_vector.transform.translation.x
        )
        args["goal_pub"].publish(goal)
        if args["timer"] > 40:
            args["saved_goal"] = goal
            args["timer"] = 0
            return (MOVE_THROUGH_GATE,0)


class MOVE_THROUGH_GATE(State):
    def execute(args):
        args["goal_pub"].publish(args["saved_goal"])
        if args["timer"] > 80:
            args["timer"] = 0
            args["saved_goal"] = None
            return (STOP,)


def callback(sm):
    sm.pub.publish(sm.currentState.__name__)


def checkTolerance(current, wanted):
    tolerance = 0.1
    return current < wanted + tolerance and current > wanted - tolerance


def mission():
    rospy.init_node("mission_controller", anonymous=True)


    sm = StateMachine(
        SUBMERGE,
        STOP,
        [STOP, SUBMERGE, MOVE_TO_GATE, MOVE_THROUGH_GATE],
        callback,
        args={
            "submerge_depth": -1,
            "timer": 0,
            "odom": None,
            "goal_pub": rospy.Publisher("wolf_control/goal", Twist, queue_size=10),
            "saved_goal": None,
            "tf_buffer": tf2_ros.Buffer(),
        },
        publisher=rospy.Publisher("wolf_control/mission_state", String, queue_size=10),
    )
    listener = tf2_ros.TransformListener(sm.args["tf_buffer"])
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        try:
            sm.args["odom"] = sm.args["tf_buffer"].lookup_transform(
                "odom", "base_link", rospy.Time(0)
            )
            stepSMs([sm])
            sm.args["timer"] += 1
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logerr("mission_code: error finding frame")
        rate.sleep()


if __name__ == "__main__":
    try:
        mission()
    except rospy.ROSInterruptException:
        pass
