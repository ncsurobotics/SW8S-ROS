import rospy
from std_msgs.msg import String


class State:
    # must return a tuple of (next state, output message)
    @staticmethod
    def execute(args) -> tuple:
        pass

    @staticmethod
    def reset():
        pass


# This class handles the state machine execution
# It takes
# initalState: the state that it will begin with
# endStates: a state, that when reached will end the execution
# states: a list of the states(their types not their strings)
# callback: a function that will be called during every step (preferrably includes a roslog and publish)
# args: dictionary of arguments that the states can use
# pub: the publisher for the statemachine


class StateMachine:
    output = None
    ended: bool = False

    def __init__(
        self,
        initialState,
        endStates,
        states,
        callback,
        args=None,
        publisher: rospy.Publisher = None,
    ):
        self.initialState = initialState
        self.currentState = initialState
        self.endStates = endStates
        if not type(self.endStates) == list:
            self.endStates = [self.endStates]
        self.callback = callback
        self.states = states
        self.pub = publisher
        self.args = args
        if self.pub == None:
            self.pub = rospy.Publisher("StateMachine", String, queue_size=10)

    # executes the current state, and returns if it was successful or not
    def step(self):
        # check if the machine is running past it's lifespan
        if self.ended:
            print('State Machine "%s" Has Ended!' % self.name)
            return False

        # process execution of state
        result: tuple = self.currentState.execute(self.args)
        if result == None:
            return

        # parse outputs and perform transition
        self.currentState = result[0]
        if len(result) > 1:
            self.output = result[1]
        # else:
        #     self.output = None

        # check if sm should end
        if self.currentState in self.endStates:
            self.ended = True

        # perform ros stuff in callback
        self.callback(self)

        return self.ended

    def reset(self):
        self.currentState = self.initialState
        self.ended = False
        self.output = None
        for s in self.states:
            s.reset()


# step all the statemachines
def stepSMs(statemachines):
    end = True
    for s in statemachines:
        if not s.ended:
            s.step()
        end = end and s.ended
    return end


# executes all the statemachines
def runSMs(statemachines, rate):
    while not (rospy.is_shutdown() or stepSMs(statemachines)):
        rate.sleep()
    return
