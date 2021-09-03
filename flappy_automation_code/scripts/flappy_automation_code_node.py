#!/usr/bin/env python
import numpy as np

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan


##===================== FLAPPY BIRD CODE STRUCTURE ==============##
# Transitions

# States
#### Go_Up
#### Go_Down
#### Go_Through
#### Adjust

# Finite State Machine

# Character
#### Flappy_Bird

# Main Code

##===================== TRANSITIONS =====================##
class Transition(object):
    """The Transition object is responsible for executing
    transitions from one state to another.
    """

    def __init__(self, to_state):
        self.to_state = to_state

    def execute(self):
        pass


##===================== STATES  =====================##
class State(object):
    """The State object is a flappy bird super class that contains all
     the methods needed by all the available states such as the
     Go_Up State, Go_Down State, Go_Through State and Adjust State.
     """

    def __init__(self, FSM):
        self.FSM = FSM
        self.x_position, self.y_position = 0, -16
        self.normalizer = 3.54999995
        self.err_x_prev, self.err_y_prev, self.err_vx_prev, self.err_vy_prev = 0, 0, 0, 0
        self.Kvx_p, self.Kvy_p, self.Kvx_d, self.Kvy_d = 0.0, 0.0, 0.0, 0.0
        self.Kax_p, self.Kay_p, self.Kax_d, self.Kay_d = 0.0, 0.0, 0.0, 0.0
        self.vx, self.vy = 0, 0
        self.ranges = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.acc = Vector3(0, 0, 0)
        self.upper_range = [[0.0, 0.0, 0.0, 0.0]]
        self.lower_range = [[0.0, 0.0, 0.0, 0.0]]
        self.primary = [[0.0, 0.0]]
        self.upper_range_sum = 0.0
        self.lower_range_sum = 0.0
        self.primary_sum = 0.0
        self.flappy_vel = rospy.Subscriber("/flappy_vel", Vector3, self.velocity_callback)
        self.laser_scan = rospy.Subscriber("/flappy_laser_scan", LaserScan, self.laser_scan_callback)
        self.pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

    def velocity_callback(self, msg):
        """The callback to receive velocity data and estimate the position.

        :param msg: velocity data

        :returns: None
        """
        self.vx = msg.x
        self.vy = msg.y
        self.estimate_position()

    def laser_scan_callback(self, msg):
        """The callback to receive and preprocess laser scan data.

        :param msg: laser scan data

        :returns: None
        """
        self.ranges = np.array(msg.ranges) / self.normalizer
        self.upper_range = self.ranges[[5, 6, 7, 8]]
        self.lower_range = self.ranges[[0, 1, 2, 3]]
        self.primary = self.ranges[[3, 5]]
        self.upper_range_sum = sum(self.upper_range)
        self.lower_range_sum = sum(self.lower_range)
        self.primary_sum = sum(self.primary)

    def estimate_position(self):
        """For estimating position from velocity.

        :param: None

        :returns: None
        """
        self.x_position += self.vx
        self.y_position += self.vy

    def enter(self):
        pass

    def execute(self):
        pass

    def compute_errors(self, err_x, err_y):
        """For computing the errors to update the velocity and acceleration using a P-D Controller.

        :param err_x: x position error
        :param err_y: y position error

        :returns: None
        """
        # compute reference velocity
        ref_vx = self.Kvx_p * err_x + self.Kvx_d * (err_x - self.err_x_prev)
        ref_vy = self.Kvy_p * err_y + self.Kvy_d * (err_y - self.err_y_prev)

        # compute velocity errors
        err_vx = ref_vx - self.vx
        err_vy = ref_vy - self.vy

        # compute reference acceleration
        ref_ax = self.Kax_p * err_vx + self.Kax_d * (err_vx - self.err_vx_prev)
        ref_ay = self.Kay_p * err_vy + self.Kay_d * (err_vy - self.err_vy_prev)

        # Store errors for D-Control
        self.err_x_prev = err_x
        self.err_y_prev = err_y
        self.err_vx_prev = err_vx
        self.err_vy_prev = err_vy

        # Publish acceleration
        self.send_acceleration(ref_ax, ref_ay)
        self.publish_acceleration()

    def set_velocity_gains(self, _Kvx_p, _Kvy_p, _Kvx_d, _Kvy_d):
        """For Setting the gains of PD-controller for velocity.

        :param _Kvx_p : proportional gain for x velocity
        :param _Kvy_p: proportional gain for y velocity
        :param _Kvx_d: derivative gain for x velocity
        :param _Kvy_d: derivative gain for y velocity

        :returns: None
        """
        self.Kvx_p, self.Kvy_p = _Kvx_p, _Kvy_p
        self.Kvx_d, self.Kvy_d = _Kvx_d, _Kvy_d

    def set_acceleration_gains(self, _Kax_p, _Kay_p, _Kax_d, _Kay_d):
        """ For Setting the gains of PD-controller for acceleration.

        :param _Kax_p : proportional gain for x acceleration
        :param _Kay_p: proportional gain for y acceleration
        :param _Kax_d: derivative gain for x acceleration
        :param _Kay_d: derivative gain for y acceleration

        :returns: None

        """
        self.Kax_p, self.Kay_p = _Kax_p, _Kay_p
        self.Kax_d, self.Kay_d = _Kax_d, _Kay_d

    def send_acceleration(self, _ax, _ay):
        """For saving the computed acceleration

        :param _ax: x - acceleration
        :param  _ay: y - acceleration

        :returns: None
        """
        self.acc = Vector3(_ax, _ay, 0)

    def publish_acceleration(self):
        """For publishing the computed acceleration to the /flappy_acc publisher

        :param: None

        :returns: None
        """
        self.pub_acc_cmd.publish(self.acc)

    def exit(self):
        pass


class Go_Up(State):
    """ For handling the go up state of flappy bird.
    """

    def __init__(self, FSM):
        super(Go_Up, self).__init__(FSM)

    def enter(self):
        """For activating all the actions needed before executing this state.

        :param: None

        :returns: None
        """
        err_x = -self.primary_sum  # slow down
        err_y = self.primary_sum + 0.05  # go up
        self.set_velocity_gains(10, 7, 0.2, 0.3)
        self.set_acceleration_gains(10, 5, 0.3, 0.4)
        super(Go_Up, self).compute_errors(err_x, err_y)

    def execute(self):
        """For executing this state

        :param: None

        :returns: None
        """
        self.enter()
        if self.ranges[3] < 0.24 or self.ranges[5] < 0.24:
            if 0 > self.y_position:
                self.FSM.to_transition("to_go_up")
            if self.y_position < -18:
                self.FSM.to_transition("to_go_up")
            elif self.y_position > 18:
                self.FSM.to_transition("to_go_down")
        elif self.ranges[3] > 0.6 and self.ranges[4] > 0.6 and self.ranges[5] > 0.6:
            self.FSM.to_transition("to_go_through")
        else:
            self.FSM.to_transition("to_adjust")

    def exit(self):
        pass


class Go_Down(State):
    """ For handling the go down state of flappy bird.
    """

    def __init__(self, FSM):
        super(Go_Down, self).__init__(FSM)

    def enter(self):
        """For activating all the actions needed before executing this state.

        :param: None

        :returns: None
        """
        err_x = -self.primary_sum  # slow down
        err_y = -self.primary_sum + 0.05  # go down
        self.set_velocity_gains(10, 7, 0.2, 0.3)
        self.set_acceleration_gains(10, 5, 0.3, 0.4)
        super(Go_Down, self).compute_errors(err_x, err_y)

    def execute(self):
        """For executing this state

        :param: None

        :returns: None
        """
        self.enter()
        if self.ranges[3] < 0.24 or self.ranges[5] < 0.24:
            if 0 < self.y_position:
                self.FSM.to_transition("to_go_down")
            if self.y_position < -18:
                self.FSM.to_transition("to_go_up")
            elif self.y_position > 18:
                self.FSM.to_transition("to_go_down")
        elif self.ranges[3] > 0.6 and self.ranges[4] > 0.6 and self.ranges[5] > 0.6:
            self.FSM.to_transition("to_go_through")
        else:
            self.FSM.to_transition("to_adjust")

    def exit(self):
        pass


class Go_Through(State):
    """ For handling the go through state of flappy bird.
    """

    def __init__(self, FSM):
        super(Go_Through, self).__init__(FSM)

    def enter(self):
        """For activating all the actions needed before executing this state.

        :param: None

        :returns: None
        """
        err_x = self.primary_sum  # move forward
        err_y = self.ranges[6] - self.ranges[2]  # regulate for symmetric duct entry
        self.set_velocity_gains(2, 1, 0.1, 0.2)
        self.set_acceleration_gains(30, 25, 2, 4)
        super(Go_Through, self).compute_errors(err_x, err_y)

    def execute(self):
        """For executing this state

       :param: None

       :returns: None
       """
        self.enter()
        if self.ranges[3] < 0.24 or self.ranges[5] < 0.24:
            if 0 < self.y_position:
                self.FSM.to_transition("to_go_down")
            elif 0 > self.y_position:
                self.FSM.to_transition("to_go_up")
            if self.y_position < -18:
                self.FSM.to_transition("to_go_up")
            elif self.y_position > 18:
                self.FSM.to_transition("to_go_down")
        elif self.ranges[3] > 0.6 and self.ranges[4] > 0.6 and self.ranges[5] > 0.6:
            self.FSM.to_transition("to_go_through")
        else:
            self.FSM.to_transition("to_adjust")

    def exit(self):
        pass


class Adjust(State):
    """ For handling the adjust state of flappy bird.
    """

    def __init__(self, FSM):
        super(Adjust, self).__init__(FSM)

    def enter(self):
        """For activating all the actions needed before executing this state.

       :param: None

       :returns: None
       """
        err_x = self.primary_sum  # move forward
        err_y = (self.upper_range_sum - self.lower_range_sum)  # adjust via imbalance to find holes
        self.set_velocity_gains(1, 0.5, 0.1, 0.2)
        self.set_acceleration_gains(30, 25, 0.3, 2)
        super(Adjust, self).compute_errors(err_x, err_y)

    def execute(self):
        """For executing this state

        :param: None

        :returns: None
        """
        self.enter()
        if self.ranges[3] < 0.24 or self.ranges[5] < 0.24:
            if 0 < self.y_position:
                self.FSM.to_transition("to_go_down")
            elif 0 > self.y_position:
                self.FSM.to_transition("to_go_up")
            if self.y_position < -18:
                self.FSM.to_transition("to_go_up")
            elif self.y_position > 18:
                self.FSM.to_transition("to_go_down")
        elif self.ranges[3] > 0.6 and self.ranges[4] > 0.6 and self.ranges[5] > 0.6:
            self.FSM.to_transition("to_go_through")
        else:
            self.FSM.to_transition("to_adjust")

    def exit(self):
        pass


##===================== FINITE STATE MACHINES =====================##
class FSM(object):
    """ This is the Finite State Machine of flappy bird.
    """

    def __init__(self, character):
        self.character = character
        self.states = {}
        self.transitions = {}
        self.current_state = None
        self.previous_state = None
        self.trans = None

    def add_transition(self, transition_name, transition):
        """For adding transition to the FSM

        :param: transition_name: The name of the transition
        :param: transition: The state to execute for the transition

        :returns: None
        """
        self.transitions[transition_name] = transition

    def add_state(self, state_name, state):
        """For adding states to the FSM

        :param: state_name: The name of the state
        :param: state: The state to be executed

        :returns: None
        """
        self.states[state_name] = state

    def set_state(self, state_name):
        """For assigning a default state to the FSM

        :param: state_name: The name of the state

        :returns: None
        """
        self.previous_state = self.current_state
        self.current_state = self.states[state_name]

    def to_transition(self, to_trans):
        """For transitioning the FSM states

        :param: to_trans: The name of the state to be transitioned

        :returns: None
        """
        self.trans = self.transitions[to_trans]

    def execute(self):
        """For executing the FSM transitions

        :param: None

        :returns: None
        """
        if self.trans:
            self.current_state.exit()
            self.trans.execute()
            self.set_state(self.trans.to_state)
            self.current_state.enter()
            self.trans = None
        self.current_state.execute()


##===================== IMPLEMENTATION =====================##
Bird = type("Bird")


class Flappy_Bird(Bird):
    """ This is the flappy bird character that specifies the states and transitions.
    """

    def __init__(self):
        self.FSM = FSM(self)

        ## STATES
        self.FSM.add_state("Go_Down", Go_Down(self.FSM))
        self.FSM.add_state("Go_Through", Go_Through(self.FSM))
        self.FSM.add_state("Adjust", Adjust(self.FSM))
        self.FSM.add_state("Go_Up", Go_Up(self.FSM))

        ## TRANSITIONS
        self.FSM.add_transition("to_go_up", Transition("Go_Up"))
        self.FSM.add_transition("to_go_down", Transition("Go_Down"))
        self.FSM.add_transition("to_go_through", Transition("Go_Through"))
        self.FSM.add_transition("to_adjust", Transition("Adjust"))

        ## DEFAULT STATE
        self.FSM.set_state("Adjust")

    def execute(self):
        self.FSM.execute()


##===================== MAIN EXECUTION =====================##

if __name__ == '__main__':
    try:
        rospy.init_node('flappy_automation_code', anonymous=True)
        flappy_bird = Flappy_Bird()
        while not rospy.is_shutdown():
            flappy_bird.execute()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("An Error Occurred")
