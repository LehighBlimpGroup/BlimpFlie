"""
Author       : Hanqing Qi
Date         : 2023-11-08 00:15:24
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-09 15:59:54
FilePath     : /GUI/simpleGUIutils.py
Description  : Some functions for the simpleGUI.py
"""

import matplotlib.widgets as widgets
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import math

# Macros
GUI_SIZE = [20, 6]
CURRENT_HEIGHT_BAR = [9, 12.5 / 6]  # X, Scale
DESIRED_HEIGHT_BAR = [10, 12.5 / 6]  # X, Scale
WALL_SENSOR_BAR = [0, 100, 20]  # X, Scale, moving average window size
BATTARY_INDICATOR = [4.2, 4.6 / 1.6, [3.8, 3.5, 3.2], 10]  # Y, Scale, level thresholds, moving average window size
FRAME_SIZE = [240, 160]
FRAME_OFFSET = [12, FRAME_SIZE[1] / GUI_SIZE[1]]
BAR_WIDTH = 1
DEBUG_HEIGHT = 2
## Colors
COLORS = {
    "red": "#FF5555",
    "green": "#50FA7B",
    "yellow": "#F1FA8C",
    "purple": "#BD93F9",
    "orange": "#FFB86C",
    "cyan": "#8BE9FD",
    "pink": "#FF79C6",
    "white": "#F8F8F2",
    "black": "#282A36",
    "gray": "#44475A",
}


def init_circles(self, add_tick: bool = True) -> None:
    """
    @description: Initialize the yaw circle
    @param       {*} self:
    @param       {bool} add_tick: If True, add ticks to the circle
    @return      {*} None
    """
    center_x, center_y = 2 + GUI_SIZE[1] / 2, GUI_SIZE[1] / 2
    dot = self.ax.scatter(center_x, center_y, s=100, color=COLORS["white"], zorder=10)
    radius = GUI_SIZE[1] / 2
    self.circle = plt.Circle((center_x, center_y), radius, fill=False, color=COLORS["white"], linewidth=3, zorder=1)
    self.ax.add_patch(self.circle)
    if add_tick:
        add_ticks_circle(self, center_x, center_y, radius)  # Pass the center and radius to the method
    self.current_yaw = self.ax.arrow(center_x, center_y, 0, 0, head_width=0.2, head_length=0.3, fc=COLORS["green"], ec=COLORS["green"], linewidth=3)
    self.desired_yaw = self.ax.arrow(center_x, center_y, 0, 0, head_width=0.2, head_length=0.3, fc=COLORS["red"], ec=COLORS["red"], linewidth=3)
    # Small dot to indicate connection
    self.connection_indicator = self.ax.scatter(center_x - 1, -0.3, s=80, color=COLORS["red"])


def init_bars(self, add_tick: bool = True) -> None:
    """
    @description: Initializes the bars
    @param       {*} self:
    @param       {bool} add_tick: If True, add ticks to the bars
    @return      {*}
    """
    # Current height bar
    self.cur_height_bar = self.ax.bar(CURRENT_HEIGHT_BAR[0] + BAR_WIDTH / 2, 0, BAR_WIDTH, color=COLORS["green"], bottom=0)
    # Desired height bar
    self.des_height_bar = self.ax.bar(DESIRED_HEIGHT_BAR[0] + BAR_WIDTH / 2, 0, BAR_WIDTH, color=COLORS["red"], bottom=0)
    # Wall sensor bar
    self.wall_sensor_bar = self.ax.bar(WALL_SENSOR_BAR[0] + BAR_WIDTH / 2, 0, BAR_WIDTH, color=COLORS["purple"], bottom=0)
    if add_tick:
        add_ticks_bar(self, CURRENT_HEIGHT_BAR[0])
        add_ticks_bar(self, DESIRED_HEIGHT_BAR[0])
        add_ticks_bar(self, WALL_SENSOR_BAR[0])
    self.current_height_value = self.ax.text(CURRENT_HEIGHT_BAR[0], 0, "", fontsize=12, color=COLORS["white"])
    self.desired_height_value = self.ax.text(DESIRED_HEIGHT_BAR[0], 0, "", fontsize=12, color=COLORS["white"])
    self.distance_value = self.ax.text(WALL_SENSOR_BAR[0], 0, "", fontsize=12, color=COLORS["white"])
    # Battery indicator
    self.battery_indicator = self.ax.barh(2, 0, 0.5, color=COLORS["green"], left=BATTARY_INDICATOR[0], zorder=2)


def init_rects(self):
    # Battery indicator
    self.bat_rect = patches.Rectangle((BATTARY_INDICATOR[0], 1.75), 1.6, 0.5, linewidth=1, edgecolor=None, facecolor=COLORS["gray"], zorder=1)
    self.ax.add_patch(self.bat_rect)
    # Frame
    frame_w = FRAME_SIZE[0] / FRAME_OFFSET[1]
    self.frame_rect = patches.Rectangle((FRAME_OFFSET[0], 0), frame_w, GUI_SIZE[1], linewidth=1.5, edgecolor=COLORS["white"], facecolor="none")
    self.ax.add_patch(self.frame_rect)
    add_ticks_rect(self)
    # ROI
    self.roi_rect = patches.Rectangle((FRAME_OFFSET[0], 0), 0, 0, linewidth=1.5, edgecolor=COLORS["yellow"], facecolor="none")
    # self.detect_rect = patches.Rectangle((0, 0), 0, 0, linewidth=2, edgecolor=COLORS["pink"], facecolor="none")\
    self.ax.add_patch(self.roi_rect)


def init_texts(self):
    # Height
    self.ax.text(CURRENT_HEIGHT_BAR[0] + BAR_WIDTH / 2, -0.4, "Current", fontsize=11, color=COLORS["green"], horizontalalignment="center")  # Current height
    self.cur_height_tx = self.ax.text(
        CURRENT_HEIGHT_BAR[0] + BAR_WIDTH, self.height_offset, 0, fontsize=11, color=COLORS["green"], horizontalalignment="right", verticalalignment="bottom"
    )  # Current height
    self.ax.text(DESIRED_HEIGHT_BAR[0] + BAR_WIDTH / 2, -0.4, "Desired", fontsize=11, color=COLORS["red"], horizontalalignment="center")  # Desired height
    self.des_height_tx = self.ax.text(
        DESIRED_HEIGHT_BAR[0] + BAR_WIDTH, self.height_offset, 0, fontsize=11, color=COLORS["red"], horizontalalignment="right", verticalalignment="bottom"
    )  # Desired height
    self.ax.text(DESIRED_HEIGHT_BAR[0], -0.7, "Height", fontsize=11, color=COLORS["white"], horizontalalignment="center")  # Height
    # Yaw
    self.ax.text(5, 1.1, "Yaw", fontsize=11, color=COLORS["white"], horizontalalignment="center")  # Yaw
    self.cur_yaw_tx = self.ax.text(5, GUI_SIZE[1] - 1.4, "Current: 0˚", fontsize=11, color=COLORS["green"], horizontalalignment="center", zorder=1)  # Current yaw
    self.des_yaw_tx = self.ax.text(5, GUI_SIZE[1] - 1.7, "Desired: 0˚", fontsize=11, color=COLORS["red"], horizontalalignment="center", zorder=1)  # Desired yaw
    # Distance
    self.ax.text(WALL_SENSOR_BAR[0] + BAR_WIDTH / 2, -0.4, "Distance", fontsize=11, color=COLORS["purple"], horizontalalignment="center")  # Wall sensor
    self.distance_tx = self.ax.text(WALL_SENSOR_BAR[0] + BAR_WIDTH, 0, 0, fontsize=11, color=COLORS["purple"], horizontalalignment="right", verticalalignment="bottom")  # Wall sensor
    # Battery
    self.ax.text(5, 2.4, "Battery", fontsize=11, color=COLORS["white"], horizontalalignment="center")  # Battery
    self.batlevel_tx = self.ax.text(5, 1.9, "?V", fontsize=11, color=COLORS["white"], horizontalalignment="center", zorder=3)  # Battery level
    # Nicla
    self.ax.text(FRAME_OFFSET[0] + 4.5, -0.4, "Nicla", fontsize=11, color=COLORS["yellow"], horizontalalignment="center")  # ROI
    self.ax.text(FRAME_OFFSET[0] + 4.5, GUI_SIZE[1], int(FRAME_SIZE[0] / 2), fontsize=11, color=COLORS["white"], horizontalalignment="center", verticalalignment="bottom")  # ROI
    self.ax.text(FRAME_OFFSET[0] + 9, GUI_SIZE[1], int(FRAME_SIZE[0]), fontsize=11, color=COLORS["white"], horizontalalignment="center", verticalalignment="bottom")  # ROI
    self.ax.text(FRAME_OFFSET[0], GUI_SIZE[1] / 2, int(FRAME_SIZE[1] / 2), fontsize=11, color=COLORS["white"], horizontalalignment="right", verticalalignment="center")  # ROI
    self.ax.text(FRAME_OFFSET[0], 0, int(FRAME_SIZE[1]), fontsize=11, color=COLORS["white"], horizontalalignment="right", verticalalignment="center")  # ROI
    self.roi_tx = self.ax.text(FRAME_OFFSET[0] + 4.5, 0.3, "(0, 0, 0, 0)", fontsize=11, color=COLORS["yellow"], horizontalalignment="center", verticalalignment="bottom")  # ROI
    # Connection
    self.connection_tx = self.ax.text(5, -0.4, "Disconnected", fontsize=11, color=COLORS["red"], horizontalalignment="center")  # Connection


def init_scrollbar(self):
    # Yaw offset
    self.yaw_offset = 0
    yaw_offset_scrollbar_ax = plt.axes([0.1, 0.15, 0.01, 0.5])
    self.yaw_offset_scrollbar = widgets.Slider(
        yaw_offset_scrollbar_ax, "yaw\noffset", -math.pi, math.pi, valinit=0, orientation="vertical", initcolor=COLORS["red"], track_color=COLORS["gray"], valstep=math.pi / 180
    )
    self.yaw_offset_scrollbar.label.set_color(COLORS["white"])
    self.yaw_offset_scrollbar.valtext.set_color(COLORS["white"])
    self.yaw_offset_scrollbar.poly.set_color(COLORS["purple"])
    self.yaw_offset_scrollbar.on_changed(self._update_yaw_offset)
    self.yaw_offset_scrollbar.ax.set_visible(False)  # Hide the scrollbar
    # Height Offset
    self.height_offset = 0
    height_offset_scrollbar_ax = plt.axes([0.125, 0.15, 0.01, 0.5])
    self.height_offset_scrollbar = widgets.Slider(height_offset_scrollbar_ax, "height\noffset", -10, 10, valinit=0, orientation="vertical", initcolor=COLORS["red"], track_color=COLORS["gray"])
    self.height_offset_scrollbar.label.set_color(COLORS["white"])
    self.height_offset_scrollbar.valtext.set_color(COLORS["white"])
    self.height_offset_scrollbar.poly.set_color(COLORS["purple"])
    self.height_offset_scrollbar.on_changed(self._update_height_offset)
    self.height_offset_scrollbar.ax.set_visible(False)  # Hide the scrollbar


def init_bottons(self):
    # Debug Toggle
    debug_bottom_ax = plt.axes([0.1, 0.05, 0.04, 0.06])
    self.debug_bottom = widgets.Button(debug_bottom_ax, "DEBUG\noff", color=COLORS["red"], hovercolor=COLORS["red"] + "AF")
    self.debug_bottom.on_clicked(self._debug_callback)
    # Vision Toggle
    vision_bottom_ax = plt.axes([0.71, 0.05, 0.05, 0.06])
    self.vision_bottom = widgets.Button(vision_bottom_ax, "NICLA\noff", color=COLORS["orange"], hovercolor=COLORS["orange"] + "AF")
    self.vision_bottom.on_clicked(self._vision_callback)
    # Wall Sensor Toggle
    wall_sensor_bottom_ax = plt.axes([0.266, 0.05, 0.05, 0.06])
    self.wall_sensor_bottom = widgets.Button(wall_sensor_bottom_ax, "Distance\noff", color=COLORS["orange"], hovercolor=COLORS["orange"] + "AF")
    self.wall_sensor_bottom.on_clicked(self._wall_sensor_callback)
    ## Wall Sensor Moving Average
    wsma_ax = plt.axes([0.266, 0.11, 0.05, 0.05])
    self.wsma = widgets.Button(wsma_ax, "Smooth\noff", color=COLORS["orange"], hovercolor=COLORS["orange"] + "AF")
    self.wsma.on_clicked(self._wsma_callback)
    # Battery Toggle
    battery_bottom_ax = plt.axes([0.366, 0.05, 0.05, 0.06])
    self.battery_bottom = widgets.Button(battery_bottom_ax, "Battery\noff", color=COLORS["orange"], hovercolor=COLORS["orange"] + "AF")
    self.battery_bottom.on_clicked(self._battery_callback)
    ## Battry Moving Average
    bma_ax = plt.axes([0.366, 0.11, 0.05, 0.05])
    self.bma = widgets.Button(bma_ax, "Smooth\noff", color=COLORS["orange"], hovercolor=COLORS["orange"] + "AF")
    self.bma.on_clicked(self._bma_callback)
    # Yaw control mode
    yaw_control_mode_ax = plt.axes([0.316, 0.05, 0.05, 0.06])
    self.yaw_control_mode_button = widgets.Button(yaw_control_mode_ax, "Yaw Mode\nGain", color=COLORS["cyan"], hovercolor=COLORS["cyan"] + "AF")
    self.yaw_control_mode_button.on_clicked(self._yaw_control_mode_callback)
    # Reconnect
    reconnect_ax = plt.axes([0.82, 0.05, 0.04, 0.06])
    self.reconnect_button = widgets.Button(reconnect_ax, "Reset", color=COLORS["yellow"], hovercolor=COLORS["yellow"] + "AF")
    self.reconnect_button.on_clicked(self._reconnect_callback)
    # Close
    close_ax = plt.axes([0.86, 0.05, 0.04, 0.06])
    self.close_button = widgets.Button(close_ax, "Close", color=COLORS["red"], hovercolor=COLORS["red"] + "AF")
    self.close_button.on_clicked(self._close_callback)


def debug_callback(self, event):
    self.debug = not self.debug
    if self.debug:
        self.debug_bottom.color = COLORS["green"]  # Change color to green when clicked
        self.debug_bottom.hovercolor = COLORS["green"] + "AF"
        self.debug_bottom.label.set_text("DEBUG\non")
        self.yaw_offset_scrollbar.ax.set_visible(True)  # Yaw scrollbar visible
        self.height_offset_scrollbar.ax.set_visible(True)  # Height scrollbar visible
    else:
        self.debug_bottom.color = COLORS["red"]  # Change color to red when unclicked
        self.debug_bottom.hovercolor = COLORS["red"] + "AF"
        self.debug_bottom.label.set_text("DEBUG\noff")
        self.yaw_offset_scrollbar.ax.set_visible(False)  # Hide the scrollbar
        self.height_offset_scrollbar.ax.set_visible(False)


def vision_callback(self, event):
    self.enable_vision = not self.enable_vision
    if self.enable_vision:
        self.vision_bottom.color = COLORS["green"]
        self.vision_bottom.hovercolor = COLORS["green"] + "AF"
        self.vision_bottom.label.set_text("NICLA\non")
    else:
        self.vision_bottom.color = COLORS["orange"]
        self.vision_bottom.hovercolor = COLORS["orange"] + "AF"
        self.vision_bottom.label.set_text("NICLA\noff")


def wall_sensor_callback(self, event):
    self.enable_wall_sensor = not self.enable_wall_sensor
    if self.enable_wall_sensor:
        self.wall_sensor_bottom.color = COLORS["green"]
        self.wall_sensor_bottom.hovercolor = COLORS["green"] + "AF"
        self.wall_sensor_bottom.label.set_text("Distance\non")
    else:
        self.wall_sensor_bottom.color = COLORS["orange"]
        self.wall_sensor_bottom.hovercolor = COLORS["orange"] + "AF"
        self.wall_sensor_bottom.label.set_text("Distance\noff")
        self.wsma.color = COLORS["orange"]
        self.wsma.hovercolor = COLORS["orange"] + "AF"
        self.wsma.label.set_text("Smooth\noff")


def battery_callback(self, event):
    self.enable_battry = not self.enable_battry
    if self.enable_battry:
        self.battery_bottom.color = COLORS["green"]
        self.battery_bottom.hovercolor = COLORS["green"] + "AF"
        self.battery_bottom.label.set_text("Battery\non")
    else:
        self.battery_bottom.color = COLORS["orange"]
        self.battery_bottom.hovercolor = COLORS["orange"] + "AF"
        self.battery_bottom.label.set_text("Battery\noff")
        self.bma.color = COLORS["orange"]
        self.bma.hovercolor = COLORS["orange"] + "AF"
        self.bma.label.set_text("Smooth\noff")


def yaw_control_mode_callback(self, event):
    self.yaw_control_mode = int(not self.yaw_control_mode)
    if self.yaw_control_mode:
        self.yaw_control_mode_button.color = COLORS["yellow"]
        self.yaw_control_mode_button.hovercolor = COLORS["yellow"] + "AF"
        self.yaw_control_mode_button.label.set_text("Yaw Mode\nMap")
    else:
        self.yaw_control_mode_button.color = COLORS["cyan"]
        self.yaw_control_mode_button.hovercolor = COLORS["cyan"] + "AF"
        self.yaw_control_mode_button.label.set_text("Yaw Mode\nGain")


def wsma_callback(self, event):
    if not self.enable_wall_sensor:
        return
    self.moving_average_distance = not self.moving_average_distance
    if self.moving_average_distance:
        self.wsma.color = COLORS["green"]
        self.wsma.hovercolor = COLORS["green"] + "AF"
        self.wsma.label.set_text("Smooth\non")
        # self.wsma.label.set_text("TODO")
    else:
        self.wsma.color = COLORS["orange"]
        self.wsma.hovercolor = COLORS["orange"] + "AF"
        self.wsma.label.set_text("Smooth\noff")


def bma_callback(self, event):
    if not self.enable_battry:
        return
    self.moving_average_battary = not self.moving_average_battary
    if self.moving_average_battary:
        self.battery_history = []
        self.bma.color = COLORS["green"]
        self.bma.hovercolor = COLORS["green"] + "AF"
        self.bma.label.set_text("Smooth\non")
        # self.bma.label.set_text("TODO")
    else:
        self.battery_history = []
        self.bma.color = COLORS["orange"]
        self.bma.hovercolor = COLORS["orange"] + "AF"
        self.bma.label.set_text("Smooth\noff")


def battery_moving_average(self, new_battery_level):
    if(len(self.battery_history) < BATTARY_INDICATOR[3]):
        self.battery_history.append(new_battery_level)
        return np.mean(self.battery_history)
    else:
        self.battery_history.pop(0)
        self.battery_history.append(new_battery_level)
        return np.mean(self.battery_history)


def distance_moving_average(self, new_distance):
    if(len(self.distance_history) < WALL_SENSOR_BAR[2]):
        self.distance_history.append(new_distance)
        return np.mean(self.distance_history)
    else:
        self.distance_history.pop(0)
        self.distance_history.append(new_distance)
        return np.mean(self.distance_history)


def add_ticks_circle(self, center_x: float, center_y: float, radius: float) -> None:
    """
    @description: Add ticks to the yaw circle
    @param       {*} self:
    @param       {float} center_x: The x value of the center of the circle
    @param       {float} center_y: The y value of the center of the circle
    @param       {float} radius: The radius of the circle
    @return      {*} None
    """
    emphasized_angles = [0, 90, 180, 270]

    for angle in range(0, 360, 5):
        radian = np.radians(angle)
        start_x = center_x + radius * np.cos(radian)
        start_y = center_y + radius * np.sin(radian)
        if angle in emphasized_angles:
            end_x = center_x + (radius - 0.4) * np.cos(radian)
            end_y = center_y + (radius - 0.4) * np.sin(radian)
            self.ax.plot([start_x, end_x], [start_y, end_y], color=COLORS["white"], linewidth=2)
        else:
            end_x = center_x + (radius - 0.2) * np.cos(radian)
            end_y = center_y + (radius - 0.2) * np.sin(radian)
            self.ax.plot([start_x, end_x], [start_y, end_y], color=COLORS["white"], linewidth=1.5)


def add_ticks_bar(self, left_edge: float) -> None:
    """
    @description: Add ticks to the bar
    @param       {*} self:
    @param       {float} left_edge: The x value of the left edge of the bar
    @return      {*} None
    """
    # Define how many ticks and their positions
    num_ticks = 50  # Total number of ticks
    tick_positions = np.linspace(0, GUI_SIZE[1], num_ticks + 1)

    for i, pos in enumerate(tick_positions):
        # Calculate the tick's start and end points
        if i == 0:
            end_x = left_edge + BAR_WIDTH
            linewidth = 1.5  # Thicker line for the first tick
        elif i % 10 == 0:  # Long tick every 10 units
            end_x = left_edge + BAR_WIDTH / 3
            linewidth = 1.5  # Thicker line for long ticks
        elif i % 5 == 0:  # Middle tick every 5 units
            end_x = left_edge + BAR_WIDTH * 2 / 9
            linewidth = 1.25  # Slightly thicker line for middle ticks
        else:  # Regular tick for other units
            end_x = left_edge + BAR_WIDTH / 9
            linewidth = 1  # Standard line width for regular ticks
        self.ax.plot([left_edge, end_x], [pos, pos], color=COLORS["white"], linewidth=linewidth)
    self.ax.plot([left_edge, left_edge], [0, GUI_SIZE[1]], color=COLORS["white"], linewidth=1.5)


def add_ticks_rect(self) -> None:
    self.ax.plot([FRAME_OFFSET[0] + 4.5, FRAME_OFFSET[0] + 4.5], [GUI_SIZE[1] - 0.2, GUI_SIZE[1]], color=COLORS["white"], linewidth=1.5)
    self.ax.plot([FRAME_OFFSET[0] + 4.5, FRAME_OFFSET[0] + 4.5], [0, 0.2], color=COLORS["white"], linewidth=1.5)
    self.ax.plot([FRAME_OFFSET[0], FRAME_OFFSET[0] + 0.2], [GUI_SIZE[1] / 2, GUI_SIZE[1] / 2], color=COLORS["white"], linewidth=2)
    self.ax.plot([FRAME_OFFSET[0] + 9 - 0.2, FRAME_OFFSET[0] + 9], [GUI_SIZE[1] / 2, GUI_SIZE[1] / 2], color=COLORS["white"], linewidth=1.5)
    self.ax.plot([FRAME_OFFSET[0] + 4.4, FRAME_OFFSET[0] + 4.6], [GUI_SIZE[1] / 2, GUI_SIZE[1] / 2], color=COLORS["white"], linewidth=1.5)
    self.ax.plot([FRAME_OFFSET[0] + 4.5, FRAME_OFFSET[0] + 4.5], [GUI_SIZE[1] / 2 - 0.1, GUI_SIZE[1] / 2 + 0.1], color=COLORS["white"], linewidth=1.5)


def update_yaw_offset(self, val):
    self.yaw_offset = val
    cur_yaw_temp = np.degrees(self.cur_yaw) % 360
    des_yaw_temp = np.degrees(self.des_yaw) % 360
    self.cur_yaw_tx.set_text(f"Current: {cur_yaw_temp:.2f}˚")  # Current yaw
    self.des_yaw_tx.set_text(f"Desired: {des_yaw_temp:.2f}˚")  # Desired yaw


def update_height_offset(self, val):
    self.height_offset = val
    cur_height_temp = (self.cur_height) / CURRENT_HEIGHT_BAR[1]
    des_height_temp = (self.des_height + self.height_offset) / DESIRED_HEIGHT_BAR[1]
    self.cur_height_tx.set_text(f"C {self.cur_height:.2f}")  # Current height
    self.cur_height_tx.set_position((CURRENT_HEIGHT_BAR[0] + BAR_WIDTH, cur_height_temp))  # , horizontalalignment='right')
    self.des_height_tx.set_text(f"D {(self.des_height + self.height_offset):.2f}")
    self.des_height_tx.set_position((DESIRED_HEIGHT_BAR[0] + BAR_WIDTH, des_height_temp))  # , horizontalalignment='right')


def angle_to_coordinates(self, radians: float) -> tuple:
    """
    @description: Convert an angle to coordinates on the circle
    @param       {*} self: -
    @param       {float} radians: The angle in radians
    @return      {tuple} (x, y) coordinates
    """
    radius = self.circle.get_radius() * 0.88
    x = radius * np.cos(radians)
    y = radius * np.sin(radians)
    return x, y