"""
Author       : Hanqing Qi
Date         : 2023-11-11 14:47:43
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-13 12:03:15
FilePath     : /ModSender/gui/sensorGUI2.py
Description  : The GUI for bicopter control V3
"""

import matplotlib.pyplot as plt
from sensorGUI2utils import *
import time


class SimpleGUI:
    def __init__(self, enable_gui = True, id: int = 0, callback_reconnect=None, callback_close=None, callback_send_flags=None):
        self.enable_gui = enable_gui
        self.id = id
        self.callback_reconnect = callback_reconnect if callback_reconnect is not None else self.dummy_debug_callback
        self.callback_close = callback_close if callback_close is not None else self.dummy_debug_callback
        self.callback_send_flags = callback_send_flags if callback_send_flags is not None else self.dummy_debug_callback
        if not self.enable_gui:
            return
        # Plotting initialization
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=GS)
        self.ax.set_facecolor(C["k"])  # Set background color
        self.fig.patch.set_facecolor(C["k"])  # Set background color
        self.ax.set_xlim(0, GS[0])
        self.ax.set_ylim(0, GS[1])
        self.ax.set_aspect("equal", "datalim")  # Set aspect ratio
        self.ax.set_xticks([])  # Remove x ticks
        self.ax.set_yticks([])  # Remove y ticks
        self.ax.axis("off")  # Remove axis

        init_yaw(self)
        init_height(self)
        init_variables(self)
        init_battery(self)
        init_distance(self)
        init_connection_status(self)
        init_buttons(self)

    def update(self, cur_yaw: float = 0, des_yaw: float = 0, cur_height: float = 0, des_height: float = 0, battery: float = 0, distance: float = 0, connection_status: bool = False):
        if not self.enable_gui:
            return
        if not self.user_toggled_connect:
            connection_status = False
        update_yaw(self, cur_yaw, des_yaw)
        update_height(self, cur_height, des_height)
        update_battery(self, battery)
        update_distance(self, distance)
        update_connection_status(self, connection_status)


    def sleep(self, t: float = 0.05):
        if self.enable_gui:
            plt.pause(t)
        else:
            time.sleep(t)

    def dummy_debug_callback(self, id, flags=None):
        print("Dummy debug callback called on GUI %d" % id)
        if flags is not None:
            print("Flags: ", flags)

if __name__ == "__main__":
    mygui = SimpleGUI(True)
    import math

    for i in range(101):
        mygui.update(
            cur_yaw=math.pi * i / 100,
            des_yaw=- math.pi * i / 100,
            cur_height=15 * i / 100,
            des_height=15 * (1 - i / 100),
            battery=4.5 * i / 100,
            distance=400 * i / 100,
            connection_status=True,
        )
        plt.pause(0.25)
    plt.ioff()
    plt.show()
