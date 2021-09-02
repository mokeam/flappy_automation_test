#!/usr/bin/env python
import numpy as np

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan


##===================== FLAPPY CODE STRUCTURE ==============##
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
    def __init__(self, to_state):
        self.to_state = to_state

    def execute(self):
        # print("Transitioning...")
        pass


##===================== STATES  =====================##
class State(object):
    def __init__(self, FSM):
        # print("Entered Main State Constructor!")
        self.FSM = FSM
        self.x_position = 0
        self.y_position = -16
        self.err_x_prev = 0
        self.err_y_prev = 0
        self.err_vx_prev = 0
        self.err_vy_prev = 0
        self.Kvx_p = 0.0
        self.Kvy_p = 0.0
        self.Kvx_d = 0.0
        self.Kvy_d = 0.0
        self.Kax_p = 0.0
        self.Kay_p = 0.0
        self.Kax_d = 0.0
        self.Kay_d = 0.0
        self.vx = 0
        self.vy = 0
        self.ranges = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ])
        self.acc = Vector3(0, 0, 0)
        self.upper = [[0.0, 0.0, 0.0, 0.0]]
        self.lower = [[0.0, 0.0, 0.0, 0.0]]
        self.primary = [[0.0, 0.0]]
        self.upper_sum = 0.0
        self.lower_sum = 0.0
        self.primary_sum = 0.0
        self.flappy_vel = rospy.Subscriber("/flappy_vel", Vector3, self.velocity_callback)
        self.laser_scan = rospy.Subscriber("/flappy_laser_scan", LaserScan, self.laser_scan_callback)
        self.pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

    def velocity_callback(self, msg):
        self.vx = msg.x
        self.vy = msg.y
        self.position_estimator()

    def laser_scan_callback(self, msg):
        self.ranges = np.array(msg.ranges) / 3.54999995
        self.upper = self.ranges[[5, 6, 7, 8]]
        self.lower = self.ranges[[0, 1, 2, 3]]
        self.primary = self.ranges[[3, 5]]
        self.upper_sum = sum(self.upper)
        self.lower_sum = sum(self.lower)
        self.primary_sum = sum(self.primary)

    def position_estimator(self):
        self.x_position += self.vx
        self.y_position += self.vy

    def enter(self):
        print("Super Enter")

    def execute(self):
        print("Super Execute")

    def compute_errors(self, err_x, err_y):
        # compute velocity input
        ref_vx = self.Kvx_p * err_x + self.Kvx_d * (err_x - self.err_x_prev)
        ref_vy = self.Kvy_p * err_y + self.Kvy_d * (err_y - self.err_y_prev)

        # compute difference between target and current velocity
        err_vx = ref_vx - self.vx
        err_vy = ref_vy - self.vy

        # compute acceleration input
        ref_ax = self.Kax_p * err_vx + self.Kax_d * (err_vx - self.err_vx_prev)
        ref_ay = self.Kay_p * err_vy + self.Kay_d * (err_vy - self.err_vy_prev)

        # save previous errors for Derivative control
        self.err_x_prev = err_x
        self.err_y_prev = err_y
        self.err_vx_prev = err_vx
        self.err_vy_prev = err_vy

        self.send_acceleration(ref_ax, ref_ay)
        self.publish_acceleration()

    def set_velocity_gains(self, _Kvx_p, _Kvy_p, _Kvx_d, _Kvy_d):
        """
        Set gains of velocity PD-controller.
        """
        self.Kvx_p, self.Kvy_p = _Kvx_p, _Kvy_p
        self.Kvx_d, self.Kvy_d = _Kvx_d, _Kvy_d

    def set_acceleration_gains(self, _Kax_p, _Kay_p, _Kax_d, _Kay_d):
        """
        Set gains of acceleration PD-controller.
        """
        self.Kax_p, self.Kay_p = _Kax_p, _Kay_p
        self.Kax_d, self.Kay_d = _Kax_d, _Kay_d

    def send_acceleration(self, _ax, _ay):
        self.acc = Vector3(_ax, _ay, 0)

    def publish_acceleration(self):
        self.pub_acc_cmd.publish(self.acc)

    def exit(self):
        print("Super Exit")


class Go_Up(State):
    def __init__(self, FSM):
        super(Go_Up, self).__init__(FSM)
        # print("Entered Go up State")

    def enter(self):
        # print("Preparing to go up")
        err_x = -self.primary_sum  # slow down
        err_y = self.primary_sum  # go up
        self.set_velocity_gains(10, 3, 0.2, 0.3)
        self.set_acceleration_gains(10, 3, 0.3, 0.3)
        super(Go_Up, self).compute_errors(err_x, err_y)

    def execute(self):
        self.enter()
        # print("Going up")
        if self.ranges[3] < 0.26 or self.ranges[5] < 0.26:
            if 0 > self.y_position:
                self.FSM.to_transition("to_go_up")
            if self.y_position < -35:
                self.FSM.to_transition("to_go_up")
            elif self.y_position > 35:
                self.FSM.to_transition("to_go_down")
        elif self.ranges[3] > 0.6 and self.ranges[4] > 0.6 and self.ranges[5] > 0.6:
            self.FSM.to_transition("to_go_through")
        else:
            self.FSM.to_transition("to_adjust")

    def exit(self):
        pass


class Go_Down(State):
    def __init__(self, FSM):
        super(Go_Down, self).__init__(FSM)

    def enter(self):
        err_x = -self.primary_sum  # slow down
        err_y = -self.primary_sum  # go down
        self.set_velocity_gains(10, 3, 0.2, 0.3)
        self.set_acceleration_gains(10, 3, 0.3, 0.3)
        super(Go_Down, self).compute_errors(err_x, err_y)

    def execute(self):
        self.enter()
        if self.ranges[3] < 0.26 or self.ranges[5] < 0.26:
            if 0 < self.y_position:
                self.FSM.to_transition("to_go_down")
            if self.y_position < -35:
                self.FSM.to_transition("to_go_up")
            elif self.y_position > 35:
                self.FSM.to_transition("to_go_down")
        elif self.ranges[3] > 0.6 and self.ranges[4] > 0.6 and self.ranges[5] > 0.6:
            self.FSM.to_transition("to_go_through")
        else:
            self.FSM.to_transition("to_adjust")

    def exit(self):
        pass


class Go_Through(State):
    def __init__(self, FSM):
        super(Go_Through, self).__init__(FSM)

    def enter(self):
        err_x = self.primary_sum  # advance
        err_y = self.ranges[6] - self.ranges[2]  # regulate for symmetric duct entry
        self.set_velocity_gains(1, 1, 0.1, 0.2)
        self.set_acceleration_gains(30, 23, 2, 3)
        super(Go_Through, self).compute_errors(err_x, err_y)

    def execute(self):
        self.enter()
        if self.ranges[3] < 0.26 or self.ranges[5] < 0.26:
            if 0 < self.y_position:
                self.FSM.to_transition("to_go_down")
            elif 0 > self.y_position:
                self.FSM.to_transition("to_go_up")
            if self.y_position < -35:
                self.FSM.to_transition("to_go_up")
            elif self.y_position > 35:
                self.FSM.to_transition("to_go_down")
        elif self.ranges[3] > 0.6 and self.ranges[4] > 0.6 and self.ranges[5] > 0.6:
            self.FSM.to_transition("to_go_through")
        else:
            self.FSM.to_transition("to_adjust")

    def exit(self):
        pass


class Adjust(State):
    def __init__(self, FSM):
        super(Adjust, self).__init__(FSM)

    def enter(self):
        err_x = self.primary_sum  # advance
        err_y = self.upper_sum - self.lower_sum  # adjust via imbalance to find holes
        self.set_velocity_gains(1, 0.5, 0.1, 0.2)
        self.set_acceleration_gains(30, 23, 0.3, 1)
        super(Adjust, self).compute_errors(err_x, err_y)

    def execute(self):
        self.enter()
        if self.ranges[3] < 0.26 or self.ranges[5] < 0.26:
            if 0 < self.y_position:
                self.FSM.to_transition("to_go_down")
            elif 0 > self.y_position:
                self.FSM.to_transition("to_go_up")
            if self.y_position < -35:
                self.FSM.to_transition("to_go_up")
            elif self.y_position > 35:
                self.FSM.to_transition("to_go_down")
        elif self.ranges[3] > 0.6 and self.ranges[4] > 0.6 and self.ranges[5] > 0.6:
            self.FSM.to_transition("to_go_through")
        else:
            self.FSM.to_transition("to_adjust")

    def exit(self):
        pass


##===================== FINITE STATE MACHINES =====================##
class FSM(object):
    def __init__(self, character):
        self.character = character
        self.states = {}
        self.transitions = {}
        self.current_state = None
        self.previous_state = None
        self.trans = None

    def add_transition(self, transition_name, transition):
        self.transitions[transition_name] = transition

    def add_state(self, state_name, state):
        self.states[state_name] = state

    def set_state(self, state_name):
        self.previous_state = self.current_state
        self.current_state = self.states[state_name]

    def to_transition(self, to_trans):
        self.trans = self.transitions[to_trans]

    def execute(self):
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


if __name__ == '__main__':
    try:
        rospy.init_node('flappy_automation_code', anonymous=True)
        flappy_bird = Flappy_Bird()
        while not rospy.is_shutdown():
            flappy_bird.execute()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("An Error Occurred")