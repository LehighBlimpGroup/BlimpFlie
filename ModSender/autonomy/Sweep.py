import time
from math import atan2, sin, cos

import numpy as np

from ModSender.autonomy.Autonomous import Autonomous




class DeterministicWalk(Autonomous):



    def __init__(self, forward_force=0.15, min_distance=550, des_z=5):
        # Constants
        self.forward_zig_zag = 1
        self.zz_counter = 0
        self.forward_force = forward_force
        self.min_distance = min_distance
        self.des_z = des_z
        self.time_backward = 2
        self.time_rotate = 5
        self.yaw = 0
        # Variable
        self.des_yaw = np.radians(295)
        self.distance = 700
        self.step_zig_zag = np.radians(20)
        self.zig_yaw = 45

        # current action
        self.current_action = 0

        # actions
        self.actions = [self._action_move_forward, self._action_move_backward, self._action_rotate, self._action_wait]

    def begin(self):
        self._restart_timer()


    def _choose_action(self, feedback):
        # Variables to make decisions
        distance = feedback[2]  # Distance from the sonar
        self.distance = distance
        time_elapsed = self._time_elapsed()
        self.yaw = feedback[1]


        SWITCH_TIME = 30
        # ---------- Switch actions based on timer and distance -----------
        # if self.current_action == 0 and (distance < self.min_distance or time_elapsed > SWITCH_TIME):
        if self.current_action == 0 and time_elapsed > SWITCH_TIME:
            self.current_action = 2  # Flip
            self._restart_timer()

        elif self.current_action == 2:
            self.current_action = 3  # Wait

        elif self.current_action == 3 and time_elapsed > self.time_rotate:
            self.current_action = 0  # Move forward
            self._restart_timer()

        print("Current action: ", self.current_action)
        return self.actions[self.current_action]

    def execute(self, feedback):
        # Select the current action function
        action = self._choose_action(feedback)

        return action()

    def _restart_timer(self):
        self.time_elapse = time.time()

    def _time_elapsed(self):
        return time.time() - self.time_elapse

    def _action_move_forward(self):
        yaw = self.yaw
        #yaw = atan2(sin(yaw), cos(yaw))  # bound angle from -pi to pi
        #wall detection
        diff = abs(self.des_yaw - self.yaw)
        if self.distance < self.min_distance and (diff < np.pi or diff > ):
            #self.zig_yaw = -self.zig_yaw
            self.des_yaw = self.des_yaw + np.pi # + 1 * self.step_zig_zag * self.forward_zig_zag
            '''
        if self.distance < self.min_distance :#and self.yaw > np.pi/2:
            self.zig_yaw = -self.zig_yaw
            self.des_yaw = np.radians(100 + self.zig_yaw)  # - 1 * self.step_zig_zag * self.forward_zig_zag
'''
        return self.forward_force, self.des_z, self.des_yaw

    def _action_move_backward(self):
        return -1.5 * self.forward_force, self.des_z, self.des_yaw

    def _action_rotate(self):
        # self.des_yaw += np.random.uniform(0, np.pi) + np.pi / 2
        yaw = self.yaw
        yaw = atan2(sin(yaw), cos(yaw))  # bound angle from -pi to pi

        if yaw > 0: # 90
            self.des_yaw = np.radians(280 + self.zig_yaw)# + 1 * self.step_zig_zag * self.forward_zig_zag
        else:
            self.des_yaw = np.radians(100 + self.zig_yaw)# - 1 * self.step_zig_zag * self.forward_zig_zag

        return 0, self.des_z, self.des_yaw

    def _action_wait(self):
        return 0, self.des_z, self.des_yaw