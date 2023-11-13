import time
from math import atan2, sin, cos, radians, degrees

import numpy as np

from autonomy.Behavior import Behavior
from autonomy.Walk import Walk

# Constants for the behavior
EAST_WEST_TIME = 15  # seconds for heading east or west
NUM_ZIGZAGS = 8
ZIGZAG_ANGLE = 90  # degrees
SAFETY_TIME = 10  # seconds to avoid obstacles
Z_LEVEL_CHANGE = 0.5  # change in Z level after each zigzag
WALL_DISTANCE_THRESHOLD = 1.0  # Threshold distance to the wall

class ZigZagWalk(Walk):
    def __init__(self, *args, **kwargs):
        super(ZigZagWalk, self).__init__(*args, **kwargs)
        self.start_time = time.time()
        self.zigzag_count = 0
        self.heading_east = True  # start by heading east
        self.in_zigzag = False

    def _choose_action(self, feedback):
        distance = feedback[2]  # Distance from the sonar
        self.yaw = feedback[1]
        time_elapsed = time.time() - self.start_time

        # Check for wall proximity during zigzag
        if self.in_zigzag and self._is_facing_wall(self.yaw, distance):
            self.in_zigzag = False
            self.heading_east = not self.heading_east
            self.start_time = time.time()
            return self._east_west_action(feedback, distance)

        # Switch from east/west to zigzag or vice versa
        if not self.in_zigzag and time_elapsed > EAST_WEST_TIME:
            self.in_zigzag = True
            self.zigzag_count = 0
            self.start_time = time.time()
        elif self.in_zigzag and self.zigzag_count >= NUM_ZIGZAGS:
            self.in_zigzag = False
            self.heading_east = not self.heading_east
            self.start_time = time.time()

        if self.in_zigzag:
            return self._zigzag_action(feedback)
        else:
            return self._east_west_action(feedback, distance)

    def _zigzag_action(self, feedback):
        time_elapsed = time.time() - self.start_time

        if time_elapsed > SAFETY_TIME:
            self.zigzag_count += 1
            self.start_time = time.time()
            self.des_z += Z_LEVEL_CHANGE
            self.des_yaw = np.radians((self.zigzag_count % 2) * ZIGZAG_ANGLE)
            return 0, self.des_z, self.des_yaw

        return self.actions[self.current_action]

    def _east_west_action(self, feedback, distance):
        if distance < self.min_distance:
            self.current_action = 2  # Rotate
            self._restart_timer()
        else:
            self.current_action = 0  # Move forward
            self.des_yaw = np.radians(0 if self.heading_east else 180)
            return self.actions[self.current_action]

    def _is_facing_wall(self, yaw, distance):
        # Check if the robot is facing towards the east or west wall and close to it
        is_facing_east = abs(self._angle_bounded(yaw) - np.radians(90)) < radians(ANGLE_THRESH)
        is_facing_west = abs(self._angle_bounded(yaw) - np.radians(270)) < radians(ANGLE_THRESH)
        return (is_facing_east or is_facing_west) and distance < WALL_DISTANCE_THRESHOLD

    def _angle_bounded(self, angle):
        return atan2(sin(angle), cos(angle))