import time
from math import atan2, sin, cos

import numpy as np

from autonomy.Behavior import Behavior
from autonomy.Walk import Walk


class ZigZagWalk(Walk):


    def _choose_action(self, feedback):
        # Variables to make decisions
        distance = feedback[2]  # Distance from the sonar
        time_elapsed = self._time_elapsed()
        self.yaw = feedback[1]


        SWITCH_TIME = 5
        # ---------- Switch actions based on timer and distance -----------
        if self.current_action == 0 and (distance < self.min_distance or time_elapsed > SWITCH_TIME):
            self.current_action = 1  # Move backwards
            self._restart_timer()
            self.zz_counter += 1

            if self.zz_counter > 8:
                self.zz_counter = 0
                self.forward_zig_zag *= -1

        elif self.current_action == 1 and time_elapsed > self.time_backward:
            self.current_action = 2  # Rotate
            self._restart_timer()
        elif self.current_action == 2:
            self.current_action = 3  # Wait
        elif self.current_action == 3 and time_elapsed > self.time_rotate:
            self.current_action = 0  # Move forward
            self._restart_timer()

        print("Current action: ", self.current_action)
        return self.actions[self.current_action]


    def _action_rotate(self):
        pass
        # self.des_yaw += np.random.uniform(0, np.pi) + np.pi / 2
        yaw = self.yaw
        yaw = atan2(sin(yaw), cos(yaw))  # bound angle from -pi to pi

        if yaw > 0: # 90
            self.des_yaw = np.radians(275+3) + 1 * self.step_zig_zag * self.forward_zig_zag
        else:
            self.des_yaw = np.radians(90+3) - 1 * self.step_zig_zag * self.forward_zig_zag


        return 0, self.des_z, self.des_yaw

