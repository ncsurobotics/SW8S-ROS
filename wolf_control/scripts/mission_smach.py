#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String
import tf2_ros
import math
import smach

# Using smach library to implement a basic state machine that functions as mission.py
# http://wiki.ros.org/smach
# roslaunch wolf_bringup state_machine_viewer.launch to run this statemachine
# there is also smach_viewer to generate an overview, but that is broken for noetic (python 3)
# Each state is it's own class 
# The states are connected by "transitions" dictionary, {return value : next state id}
# data passed between states needs to be declared as userdata before starting the statemachine
# as "remapping" dictionary, {state input_key : userdata variable}

# a list of states
# mostly needed for RequestOdom and RequestGate where they use this list to go back
# might as well keep all the states here
gate_sequence_outcomes = ['Start','Toward','Through','Complete','Finish'] 
requests = ['RequestOdom','RequestGate']

def mission():
    rospy.init_node('mission_state_machine', anonymous=True)
    state_pub = rospy.Publisher('wolf_control/mission_state', String, queue_size=10)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10)
    # statemachine container
    sm = smach.StateMachine(outcomes=[gate_sequence_outcomes[4]])
    # userdata that will be passed between states
    sm.userdata.sm_timer_o = 0
    sm.userdata.sm_sub_depth = -1
    sm.userdata.sm_state = 0
    sm.userdata.sm_goal = 0

    # adding states to the state machine
    with sm:
        # Start: submerge to sm_sub_depth
        # -> 'RequestOdom' : return back robot odom if successful (self loop)
        # -> 'RequestGate' : attempt to find gate and enter 'Toward' after
        smach.StateMachine.add('Start',start_gate(),
                                transitions={requests[0]:'RequestOdom',
                                            requests[1]:'RequestGate'},
                                remapping={'sub_depth':'sm_sub_depth',
                                            'odom':'sm_odom',
                                            'state_out':'sm_state',}
                                )
        # Toward: finding the gate location and move towards it
        # -> 'Through' : finishes finding the location
        # -> 'RequestGate' : requesting gate location, return back if successful (self loop)
        smach.StateMachine.add('Toward',gate_towards(),
                                transitions={gate_sequence_outcomes[2]:'Through',
                                            requests[1]:'RequestGate'},
                                remapping={ 'sub_depth':'sm_sub_depth',
                                            'odom_gate': 'sm_odom',
                                            'save_goal':'sm_goal',
                                            'state_in':'sm_state',
                                            'state_out':'sm_state',}
                                )
        # Through: gate location is fixed, moving through
        # -> 'Through' : still moving
        # -> 'Complete' : finished moving
        smach.StateMachine.add('Through',gate_through(),
                                transitions={gate_sequence_outcomes[2]:'Through',
                                            gate_sequence_outcomes[3]:'Complete'},
                                remapping={'saved_goal':'sm_goal',
                                            'save_goal':'sm_goal',}
                                )
        # Complete: moved through gate, stop
        # -> 'Finish' : exit state machine
        smach.StateMachine.add('Complete',gate_complete(),
                                transitions={gate_sequence_outcomes[4]:gate_sequence_outcomes[4]},
                                remapping={'sub_depth':'sm_sub_depth'})
        # provides robot position relative to base link
        # -> self : request again
        # -> in state : state that requested robot position
        smach.StateMachine.add('RequestOdom',lookup_odom_baselink(),
                                transitions={gate_sequence_outcomes[0]:'Start',
                                            requests[0]:'RequestOdom'},
                                remapping={'state_in':'sm_state',
                                            'odom_out':'sm_odom'}
                                )
        # provides robot position relative to gate location
        # -> self : request again
        # -> in state : state that requested robot position
        smach.StateMachine.add('RequestGate',lookup_odom_gate(),
                                transitions={gate_sequence_outcomes[1]:'Toward',
                                            requests[1]:'RequestGate'},
                                remapping={'state_in':'sm_state',
                                            'odom_out':'sm_odom'}
                                )                                    
    while not rospy.is_shutdown():
        try:
            odom: TransformStamped = tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
            sm.userdata.sm_odom = odom      
            # enter the state machine, break once it finishes  
            outcome = sm.execute()
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("SM: mission_code: error finding frame")
        rate.sleep()

### States
## starting the gate sequence [0]
class start_gate(smach.State):
    
    def __init__(self,outcomes=[requests[1],
                                requests[0]], 
                        input_keys=['sub_depth','odom'], 
                        output_keys=['goal','saved_goal','state_out']):
        super().__init__(outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.goal_pub = rospy.Publisher('wolf_control/goal', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
    def execute(self, ud):
        self.rate.sleep()
        sub_depth = ud.sub_depth
        odom = ud.odom
        ud.state_out = 0

        goal = Twist()
        goal.linear.z = sub_depth
        self.goal_pub.publish(goal)
        ud.goal = goal
        if self.checkTolerance(odom.transform.translation.z, sub_depth):
            ud.saved_goal = None
            ud.state_out = 1
            return requests[1] # 'find'
        return requests[0] # 'start'

    def checkTolerance(self,current, wanted):
        tolerance = 0.1
        return current < wanted + tolerance and current > wanted - tolerance


# currently it functions as 'MOVE_TO_GATE' in mission.py
class gate_towards(smach.State):
    def __init__(self, outcomes=[gate_sequence_outcomes[2],
                                 requests[1]],
                        input_keys=['sub_depth','odom_gate','state_in'],
                        output_keys=['save_goal','state_out']):
        super().__init__(outcomes=outcomes,input_keys=input_keys,output_keys=output_keys)
        self.goal_pub = rospy.Publisher('wolf_control/goal', Twist, queue_size=10)
        self.timer = 0
        self.rate = rospy.Rate(10)
    def execute(self, ud):
        self.rate.sleep()
        gate_vector = ud.odom_gate
        sub_depth = ud.sub_depth
        ud.state_out = 1

        goal = Twist()
        goal.linear.x = gate_vector.transform.translation.x * 1.3
        goal.linear.y =  gate_vector.transform.translation.y
        goal.linear.z = sub_depth
        goal.angular.z = -math.atan2(gate_vector.transform.translation.y, gate_vector.transform.translation.x)
        self.goal_pub.publish(goal)
        if self.timer > 40:
            ud.save_goal = goal
            return gate_sequence_outcomes[2]
        self.timer += 1
        return requests[1]

# currently functions as 'MOVE_THROUGH_GATE' in mission.py
class gate_through(smach.State):
    def __init__(self, outcomes=[gate_sequence_outcomes[2],gate_sequence_outcomes[3]],
                        input_keys = ['saved_goal'],
                        output_keys = ['save_goal']):
        super().__init__(outcomes=outcomes,input_keys=input_keys,output_keys=output_keys)
        self.goal_pub = rospy.Publisher('wolf_control/goal', Twist, queue_size=10)
        self.timer = 0
        self.rate = rospy.Rate(10)

    def execute(self, ud):
        self.rate.sleep()
        self.goal_pub.publish(ud.saved_goal)
        if self.timer > 80:
            self.timer = 0
            ud.save_goal = None
            return gate_sequence_outcomes[3]
        self.timer += 1
        return gate_sequence_outcomes[2]

# gate passed
# ends the gate sequence
# currently function as 'STOP' in mission.py
class gate_complete(smach.State):
    def __init__(self, outcomes=[gate_sequence_outcomes[4]], input_keys=['sub_depth']):
        super().__init__(outcomes=outcomes, input_keys=input_keys)
        self.goal_pub = rospy.Publisher('wolf_control/goal', Twist, queue_size=10)

    def execute(self, ud):
        goal = Twist()
        goal.linear.z = ud.sub_depth
        self.goal_pub.publish(goal)
        return gate_sequence_outcomes[4]

# return a robot odometry in respect to the origin
class lookup_odom_baselink(smach.State):
    def __init__(self, outcomes=[gate_sequence_outcomes[0],
                                requests[0]],
                        input_keys=['state_in'], output_keys=['odom_out']):
        super().__init__(outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    def execute(self, ud):
        try:
            odom: TransformStamped = self.tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0))
            ud.odom_out = odom       
            return gate_sequence_outcomes[ud.state_in]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("mission_code: error finding odom")
            return requests[0]

# return a gate odometry in respect to the robot
class lookup_odom_gate(smach.State):
    def __init__(self, outcomes=[gate_sequence_outcomes[1],
                                requests[1]],
                        input_keys=['state_in'], output_keys=['odom_out']):
        super().__init__(outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    def execute(self, ud):
        try:
            odom: TransformStamped = self.tf_buffer.lookup_transform("odom", "gate", rospy.Time(0))
            ud.odom_out = odom       
            return gate_sequence_outcomes[ud.state_in]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("mission_code: error finding gate")
            return requests[1]

if __name__ == '__main__':
    try:
        mission()
    except rospy.ROSInterruptException:
        pass
