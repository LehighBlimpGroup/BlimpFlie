import time
from abc import abstractmethod
from math import atan2, sin, cos

import numpy as np

from autonomy.Behavior import Behavior




class Walk(Behavior):

    def __init__(self, forward_force=0.1, min_distance=500, des_z=8):
        # Constants
        self.forward_zig_zag = 1
        self.zz_counter = 0
        self.forward_force = forward_force
        self.min_distance = min_distance
        self.des_z = des_z
        self.time_backward = 2

        self.yaw = 0
        # Variable
        self.des_yaw = np.radians(295)

        self.step_zig_zag = np.radians(10)

        # current action
        self.current_action = 0

        # actions
        self.actions = [self._action_move_forward, self._action_move_backward, self._action_rotate, self._action_wait]

    def begin(self):
        self._restart_timer()


    @abstractmethod
    def _choose_action(self, feedback):
        pass


    def execute(self, feedback):

        # Select the current action function
        action = self._choose_action(feedback)

        return action()

    def _restart_timer(self):
        self.time_elapse = time.time()

    def _time_elapsed(self):
        return time.time() - self.time_elapse

    def _action_move_forward(self):
        return self.forward_force, self.des_z, self.des_yaw

    def _action_move_backward(self):
        return -1.5 * self.forward_force, self.des_z, self.des_yaw

    @abstractmethod
    def _action_rotate(self):
        pass

    def _action_wait(self):
        return 0, self.des_z, self.des_yaw
