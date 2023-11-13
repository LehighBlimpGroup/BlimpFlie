import time
from math import atan2, sin, cos, radians, degrees

import numpy as np

from autonomy.Behavior import Behavior
from autonomy.Walk import Walk

# Number of turns before coming back
SWITCH_TIME = 3000
SAFETY_TIME = 10 # seconds
NUM_ZIGS = 2
Z_LEVEL = 0 # level height
TIME_ROTATE = 0.1  # Time to rotate in seconds
ANGLE_THRESH = 15 # Angle threshold in degrees


class ZigZagWalk(Walk):

    def _choose_action(self, feedback):
        # Variables to make decisions
        distance = feedback[2]  # Distance from the sonar
        time_elapsed = self._time_elapsed()
        self.yaw = feedback[1]


        # # ---------- Switch actions based on timer and distance -----------
        # if self.current_action == 0 and (distance < self.min_distance or time_elapsed > SWITCH_TIME):
        #     self.current_action = 1  # Move backwards
        #     self._restart_timer()
        # elif self.current_action == 1 and time_elapsed > self.time_backward:
        #     self.current_action = 2  # Rotate
        #     self._restart_timer()
        # elif self.current_action == 2:
        #     self.current_action = 3  # Wait
        # # elif self.current_action == 3 and time_elapsed > TIME_ROTATE:
        # elif self.current_action == 3 and abs(self._angle_bounded(self.yaw-self.des_yaw)) < radians(ANGLE_THRESH):
        #     self.current_action = 0  # Move forward
        #     self._restart_timer()
        #
        # print("Fwd=", self.forward_zig_zag, "ZZ=", self.zz_counter, self.actions[self.current_action].__name__, degrees(abs(self._angle_bounded(feedback[1]-self.des_yaw))),self.des_yaw,self.yaw )
        # return self.actions[self.current_action]

        # Check for wall angle in addition to distance
        if self.current_action == 0 and (feedback[2] < self.min_distance or self._check_wall_angle()) and self._time_elapsed() > SAFETY_TIME:
            self.current_action = 2  # Rotate
            self._restart_timer()
        # elif self.current_action == 3 and time_elapsed > TIME_ROTATE:
        elif self.current_action == 2: #and abs(self._angle_bounded(self.yaw-self.des_yaw)) < radians(ANGLE_THRESH):
            self.current_action = 0  # Move forward
            self._restart_timer()

        print("Fwd=", self.forward_zig_zag, "ZZ=", self.zz_counter, self.actions[self.current_action].__name__, degrees(abs(self._angle_bounded(feedback[1]-self.des_yaw))),self.des_yaw,self.yaw )
        return self.actions[self.current_action]


    def _action_rotate(self):

        # Conunt every time there is a completed cycle in the state machine
        self.zz_counter += 1

        if self.zz_counter > NUM_ZIGS:
            self.zz_counter = 0
            self.forward_zig_zag *= -1

            # Change height
            self.des_z += self.forward_zig_zag * Z_LEVEL


        # self.des_yaw += np.random.uniform(0, np.pi) + np.pi / 2
        yaw = self._angle_bounded(self.yaw)  # bound angle from -pi to pi

        if yaw < 0: # 90
            self.des_yaw = np.radians(183) + 1 * self.step_zig_zag * self.forward_zig_zag
        else:
            self.des_yaw = np.radians(3) - 1 * self.step_zig_zag * self.forward_zig_zag

        return 0, self.des_z, self.des_yaw


    def _angle_bounded(self,angle):
        """
        Bound an angle between -pi and pi
        :param angle:
        :return:
        """
        return atan2(sin(angle), cos(angle))

    def _check_wall_angle(self):
        current_yaw_deg = degrees(self._angle_bounded(self.yaw))
        if not (abs(current_yaw_deg - 183) < ANGLE_THRESH or abs(current_yaw_deg - 3) < ANGLE_THRESH):
            return True
        return False