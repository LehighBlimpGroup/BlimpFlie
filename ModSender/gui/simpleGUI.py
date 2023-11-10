"""
Author       : Hanqing Qi
Date         : 2023-11-07 15:20:18
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-09 18:15:22
FilePath     : /GUI/simpleGUI.py
Description  : The GUI for bicopter control
"""

import matplotlib.pyplot as plt
from gui.simpleGUIutils import *
import time
import sys


class SimpleGUI:
    def __init__(self, enable_gui=True, enable_wall_sensor=False, enable_vision=False, enable_battry=False):
        self.enable_gui = enable_gui
        if not enable_gui:
            return
        self.enable_wall_sensor = False
        self.enable_vision = False
        self.enable_battry = False
        self.yaw_control_mode = 0
        self.moving_average_distance = False  # Use moving average to smooth the distance sensor
        self.moving_average_battary = False  # Use moving average to smooth the battary level
        self.debug = False  # Debug
        self.flag = 1  # 1 for normal, 0 for restart, -1 for close

        # Values
        self.yaw_offset = 0
        self.height_offset = 0
        self.cur_height = 0
        self.des_height = 0
        self.cur_yaw = 0
        self.des_yaw = 0
        self.distance = 0
        self.roi = [0, 0, 0, 0]
        self.batlevel = 0
        self.connection = 0

        # Plotting initialization
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=[(GUI_SIZE[0] + 1) / 1.2, (GUI_SIZE[1] + 1) / 1])  # Create a figure and an axes
        self.fig.patch.set_facecolor("#282A36")  # Set background color
        self.ax.set_facecolor("#282A36")  # Set background color
        self.ax.set_xlim(-1, GUI_SIZE[0] + 1)  # Set x limit
        self.ax.set_ylim(-2, GUI_SIZE[1] + 1)  # Set y limit
        self.ax.set_aspect("equal", "box")  # Set aspect ratio
        self.ax.set_xticks([])  # Remove x ticks
        self.ax.set_yticks([])  # Remove y ticks
        self.ax.axis("off")  # Remove axis

        init_circles(self)  # Initialize the yaw circle
        init_scrollbar(self)  # Initialize the scrollbar
        init_bars(self)  # Initialize the bars
        init_texts(self)  # Initialize the texts
        init_bottons(self)  # Initialize the buttons
        init_rects(self)  # Initialize the
        if enable_wall_sensor:
            wall_sensor_callback(self, None)
        if enable_vision:
            vision_callback(self, None)
        if enable_battry:
            battery_callback(self, None)

    # Callback functions
    def _update_yaw_offset(self, val):
        update_yaw_offset(self, val)

    def _update_height_offset(self, val):
        update_height_offset(self, val)

    def _debug_callback(self, event):
        debug_callback(self, event)

    def _wall_sensor_callback(self, event):
        wall_sensor_callback(self, event)

    def _vision_callback(self, event):
        vision_callback(self, event)

    def _battery_callback(self, event):
        battery_callback(self, event)

    def _yaw_control_mode_callback(self, event):
        yaw_control_mode_callback(self, event)

    def _wsma_callback(self, event):
        wsma_callback(self, event)

    def _bma_callback(self, event):
        bma_callback(self, event)

    def _reconnect_callback(self, event):
        self.flag = 0
        self.yaw_offset = 0
        self.height_offset = 0
        self.cur_height = 0
        self.des_height = 0
        self.cur_yaw = 0
        self.des_yaw = 0
        self.distance = 0
        self.roi = [0, 0, 0, 0]
        self.batlevel = 0
        self.connection = 0
        self.sleep(0.1)

    def _close_callback(self, event):
        self.flag = -1

    def update_interface(
        self, cur_yaw: float = 0, des_yaw: float = 0, cur_height: float = 0, des_height: float = 0, distance: float = 0, roi: list = [0, 0, 0, 0], batlevel: float = 0, connection: bool = False
    ) -> int:
        """
        @description: Update the GUI
        @param       {*} self:
        @param       {float} cur_yaw: Current yaw angle in radians (default: 0)
        @param       {float} des_yaw: Desired yaw angle in radians (default: 0)
        @param       {float} cur_height: Current height (default: 0)
        @param       {float} des_height: Desired height (default: 0)
        @param       {float} distance: Distance sensor data (default: 0)
        @param       {list} roi: ROI data (default: [0, 0, 0, 0])
        @param       {float} batlevel: Battery level (default: 0)
        @param       {bool} connection: Connection status (default: False)
        @return      {int} status: 1 for normal, 0 for restart, -1 for close
        """
        if not self.enable_gui:
            return
        # Clear the previous drawings
        self.reset()
        if self.yaw_control_mode:  # Map
            self.cur_yaw, self.des_yaw = cur_yaw, des_yaw
        else:  # Gain
            self.cur_yaw = cur_yaw
            self.des_yaw += des_yaw
        self.cur_height, self.des_height, self.distance, self.roi, self.batlevel, self.connection = cur_height, des_height, distance, roi, batlevel, connection

        # Update the yaw circle
        cur_x, cur_y = angle_to_coordinates(self, self.cur_yaw + self.yaw_offset)
        des_x, des_y = angle_to_coordinates(self, self.des_yaw + self.yaw_offset)
        self.current_yaw = self.ax.arrow(
            self.circle.center[0],
            self.circle.center[1],
            cur_x,
            cur_y,
            head_width=0.2,
            head_length=0.3,
            fc=COLORS["green"],
            ec=COLORS["green"],
            linewidth=3,
            zorder=2,
        )
        self.desired_yaw = self.ax.arrow(
            self.circle.center[0],
            self.circle.center[1],
            des_x,
            des_y,
            head_width=0.2,
            head_length=0.3,
            fc=COLORS["red"],
            ec=COLORS["red"],
            linewidth=3,
            zorder=3,
        )
        update_yaw_offset(self, self.yaw_offset)

        # Update the heights
        self.cur_height_bar[0].set_height((cur_height) / CURRENT_HEIGHT_BAR[1] if (cur_height) > 0 else 0)
        self.des_height_bar[0].set_height((des_height + self.height_offset) / DESIRED_HEIGHT_BAR[1] if (des_height + self.height_offset) > 0 else 0)
        update_height_offset(self, self.height_offset)

        # Update the distance
        if self.enable_wall_sensor:
            if self.moving_average_distance:
                self.distance = distance_moving_average(self, self.distance)
            self.wall_sensor_bar[0].set_height(self.distance / WALL_SENSOR_BAR[1] if self.distance >= 0 else 0)
            self.distance_tx.set_text(f"{(self.distance):.2f}")
            self.distance_tx.set_position((WALL_SENSOR_BAR[0] + BAR_WIDTH, self.distance / WALL_SENSOR_BAR[1]))  # , horizontalalignment='right')
        else:
            self.wall_sensor_bar[0].set_height(0)
            self.distance_tx.set_text(0)
            self.distance_tx.set_position((WALL_SENSOR_BAR[0] + BAR_WIDTH, 0))  # , horizontalalignment='right')

        # Update the battery
        if self.enable_battry:
            self.batlevel = min(self.batlevel, 4.6)
            if self.moving_average_battary:
                self.batlevel = battery_moving_average(self, self.batlevel)
            self.battery_indicator[0].set_width(self.batlevel / BATTARY_INDICATOR[1] if self.batlevel >= 0 else 0)
            if self.batlevel > BATTARY_INDICATOR[2][0]:
                self.battery_indicator[0].set_color(COLORS["green"])
                self.batlevel_tx.set_color(COLORS["black"])
            elif self.batlevel > BATTARY_INDICATOR[2][1]:
                self.battery_indicator[0].set_color(COLORS["yellow"])
                self.batlevel_tx.set_color(COLORS["black"])
            elif self.batlevel > BATTARY_INDICATOR[2][2]:
                self.battery_indicator[0].set_color(COLORS["orange"])
                self.batlevel_tx.set_color(COLORS["black"])
            else:
                self.battery_indicator[0].set_color(COLORS["red"])
                self.batlevel_tx.set_color(COLORS["black"])
            self.batlevel_tx.set_text(f"{(self.batlevel):.2f}V")
        else:
            self.battery_indicator[0].set_width(0)
            self.batlevel_tx.set_text("?V")

        # Update the roi
        if self.enable_vision:
            # Temporarily fix for center
            center_x = roi[0]
            center_y = roi[1]
            x = center_x - roi[2] / 2 
            y = center_y - roi[3] / 2
            # Scale the roi
            w, h = roi[2], roi[3]
            # Convert the x,y on frame to x,y on GUI
            gui_x = FRAME_OFFSET[0] + x / FRAME_OFFSET[1]
            gui_y = GUI_SIZE[1] - (y + h) / FRAME_OFFSET[1] 
            gui_w = w / FRAME_OFFSET[1]
            gui_h = h / FRAME_OFFSET[1]
            self.roi_rect.set_xy((gui_x, gui_y))
            self.roi_rect.set_width(gui_w)
            self.roi_rect.set_height(gui_h)
            self.roi_tx.set_text(f"({int(x)}, {int(y)}, {int(w)}, {int(h)})")
        else:
            self.roi_rect.set_xy((0, 0))
            self.roi_rect.set_width(0)
            self.roi_rect.set_height(0)
            self.roi_tx.set_text("(0, 0, 0, 0)")

        # Update the connection
        if self.connection:
            self.connection_indicator.set_color(COLORS["green"])
            self.connection_tx.set_text("Connected")
            self.connection_tx.set_color(COLORS["green"])
        else:
            self.connection_indicator.set_color(COLORS["red"])
            self.connection_tx.set_text("Disconnected")
            self.connection_tx.set_color(COLORS["red"])
        return self.flag

    def reset(self):
        self.current_yaw.remove()
        self.desired_yaw.remove()

    def sleep(self, delay=0.05):
        if self.enable_gui:
            plt.pause(delay)
        else:
            time.sleep(delay)


if __name__ == "__main__":
    # Example of usage
    ## Flags for wall sensors, vision, and battery
    ### These can also be toggled within the GUI
    enable_wall_sensor, enable_vision, enable_battry = False, False, False
    mygui = SimpleGUI(enable_wall_sensor=enable_wall_sensor, enable_vision=enable_vision, enable_battry=enable_battry)
    while True:
        import numpy as np

        # Dummy control sim
        for i in range(1, 101):
            cur_yaw = np.pi / 100
            des_yaw = -np.pi / 100
            cur_height = i / 100 * 12.5
            des_height = i / 100 * 12.5
            distance = i / 100 * 500
            roi = [120, 80, 220, 140]
            batlevel = i / 100 * 4.6
            connection = i % 20 < 10
            flag = mygui.update_interface(cur_yaw=cur_yaw, des_yaw=des_yaw, cur_height=cur_height, des_height=des_height, distance=distance, roi=roi, batlevel=batlevel, connection=connection)
            if flag == 0:
                break
            if flag == -1:
                sys.exit()
            mygui.sleep()

        # Loop from 100 to 1
        for i in range(100, 0, -1):
            cur_yaw = np.pi / 100
            des_yaw = -np.pi / 100
            cur_height = i / 100 * 12.5
            des_height = i / 100 * 12.5
            distance = i / 100 * 500
            roi = [120, 80, 220, 140]
            batlevel = i / 100 * 4.6
            connection = i % 20 < 10
            flag = mygui.update_interface(cur_yaw=cur_yaw, des_yaw=des_yaw, cur_height=cur_height, des_height=des_height, distance=distance, roi=roi, batlevel=batlevel, connection=connection)
            if flag == 0:
                break
            if flag == -1:
                sys.exit()
            mygui.sleep()