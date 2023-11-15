"""
Author       : Hanqing Qi
Date         : 2023-10-24 16:30:43
LastEditors  : Hanqing Qi
LastEditTime : 2023-10-24 19:45:46
FilePath     : /ModSender/visualizer.py
Description  : Simple GUI for ModSender
"""
import math
import time
from math import pi
from random import random

import matplotlib.pyplot as plt
import numpy as np

import matplotlib.widgets as widgets
import matplotlib.patches as patches
from matplotlib.widgets import Slider

from parameters import BRODCAST_CHANNEL, MASTER_MAC, ROBOT_JASON


LOW_BATTERY = 3.2





class SensorGUI:
    def __init__(self, enable_gui=True, esp_now=None, robConfig = None):
        self.enable_gui = enable_gui

        # To modify from esp_now and robConfig
        self.esp_now = esp_now
        self.robConfig = robConfig

        # Flag to turn of the robot 1 is power, 0 is off
        self.toggle_power = 1
        self.toggle_joy = 0

        if not enable_gui:
            return

        # Plotting initialization
        plt.ion()

        self.fig, self.ax = plt.subplots()
        # Set background color to black
        # self.fig.patch.set_facecolor("black")
        self.ax.set_facecolor("black")

        self.ax.set_xlim(-1.1, 5)
        self.ax.set_ylim(-1.1, 1.5)
        self.ax.set_aspect("equal", "box")
        self.circle = plt.Circle((0, 0), 1, fill=False, color="white", linewidth=2)
        self.ax.add_artist(self.circle)

        # Remove x ticks and y ticks
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        # Initialize the bars
        bar_width = 0.4
        bar_bottom = -1  # Make sure the bars start from the bottom
        self.cur_height_bar = self.ax.bar(
            1.8, 0, bar_width, color="r", bottom=bar_bottom
        )
        self.des_height_bar = self.ax.bar(
            1.4, 0, bar_width, color="g", bottom=bar_bottom
        )

        self.distance_bar = self.ax.bar(
            2.4, 0.3, bar_width, color="b", bottom=bar_bottom
        )

        self.current_yaw = self.ax.arrow(
            0, 0, 0, 0, head_width=0.1, head_length=0.1, fc="r", ec="r", linewidth=3
        )
        self.desired_yaw = self.ax.arrow(
            0, 0, 0, 0, head_width=0.1, head_length=0.1, fc="g", ec="g", linewidth=3
        )

        self.current_yaw_value = self.ax.text(-0.4, 1.1, "", fontsize=12, color="white")
        self.desired_yaw_value = self.ax.text(-0.4, 1.3, "", fontsize=12, color="white")
        self.distance_value = self.ax.text(-0.4, 1.5, "", fontsize=12, color="white")
        self.battery_value = self.ax.text(-0.1, -1.5, "", fontsize=12, color="Red")

        self.current_height_value = self.ax.text(
            1.6, -1.1, "", fontsize=12, color="white"
        )
        self.desired_height_value = self.ax.text(
            1.2, -1.1, "", fontsize=12, color="white"
        )
        self.distance_value = self.ax.text(
            2.0, -1.1, "", fontsize=12, color="white"
        )

        height_label = self.ax.text(1.4, -1.2, "Height", fontsize=12, color="white")
        yaw_label = self.ax.text(-0.1, -1.2, "Yaw", fontsize=12, color="white")
        distance_label = self.ax.text(2.2, -1.2, "Distance", fontsize=12, color="white")


        ### Buttons
        self.button_ax = plt.axes([0.5, 0.9, 0.15, 0.05])  # Adjust the position and size of the button
        self.button = widgets.Button(self.button_ax, 'Reconnect')
        self.button.on_clicked(self.on_btn_reconnect_click)

        self.button_ax = plt.axes([0.65, 0.9, 0.15, 0.05])  # Adjust the position and size of the button
        self.button_flags = widgets.Button(self.button_ax, 'Flags')
        self.button_flags.on_clicked(self.on_btn_flags_click)


        ### Toggle Button
        self.toggle_ax = plt.axes([0.8, 0.9, 0.15, 0.05])  # Adjust the position and size of the toggle button
        self.toggle = widgets.CheckButtons(self.toggle_ax, ['On/Off'], [True])
        self.toggle.on_clicked(self.on_toggle_click)

        ### Toggle Button
        self.toggle_ax2 = plt.axes([0.8, 0.11, 0.15, 0.05])  # Adjust the position and size of the toggle button
        self.toggle2 = widgets.CheckButtons(self.toggle_ax2, ['Joy On/Off'], [False])
        self.toggle2.on_clicked(self.on_toggle_joy_click)


        self._draw_sliders()

        ### Case Selection Radio Buttons
        # self.radio_ax = plt.axes([0.8, 0.80, 0.15, 0.05*3])  # Adjust the position and size of the radio buttons
        # self.radio = widgets.RadioButtons(self.radio_ax, ['Case 1', 'Case 2', 'Case 3'], activecolor='red')
        # self.radio.on_clicked(self.on_radio_click)

        # Adding an enclosing square for Nicla Detection visuals
        self.enclosing_square = patches.Rectangle(
            (0, 0),  # will be updated in update_interface
            1,  # width w, will be updated in update_interface
            1,  # height h, will be updated in update_interface
            linewidth=1,
            edgecolor='yellow',
            facecolor='none',
            zorder=4  # ensure it's drawn below the Nicla Detection visuals
        )
        self.ax.add_patch(self.enclosing_square)
        # Adding the Nicla Detection Rectangle
        self.nicla_rect = patches.Rectangle(
            (0, 0),  # (x, y)
            1,  # width w
            1,  # height h
            linewidth=1,
            edgecolor='blue',
            facecolor='none',
            label='Nicla Detection',
            zorder=3  # ensure it's drawn on top
        )
        self.ax.add_patch(self.nicla_rect)

        # Set window position
        # self.set_window_position(0, y_pos)


    def _angle_to_coordinates(self, radians: float, radius: float = 1.0) -> tuple:
        """
        @description: Convert an angle to coordinates on the circle
        @param       {*} self: -
        @param       {float} radians: The angle in radians
        @param       {float} radius: The radius of the circle (default: 1.0)
        @return      {tuple} (x, y) coordinates
        """
        x = self.circle.center[0] + radius * np.cos(radians)
        y = self.circle.center[1] + radius * np.sin(radians)
        return x, y
    
    def update_nicla_box(self, x,y, w, h, max_x, max_y):
        self.nicla_rect.set_xy(((x - w/2)/max_x, (y - h/2)/max_y))
        self.nicla_rect.set_width(w/max_x)
        self.nicla_rect.set_height(h/max_y)

    def update_interface(
        self, cur_yaw: float, des_yaw: float, cur_height: float, des_height: float, distance: float,
            battery: float
    ) -> None:
        """
        @description: Update the gui interface
        @param       {*} self: -
        @param       {float} cur_yaw: Current value of yaw
        @param       {float} des_yaw: Desired value of yaw from the controller
        @param       {float} cur_height: Current value of height
        @param       {float} des_height: Desired value of height from the controller
        @return      {*} None
        """
        if not self.enable_gui:
            return

        cur_x, cur_y = self._angle_to_coordinates(cur_yaw + math.pi)
        des_x, des_y = self._angle_to_coordinates(des_yaw + math.pi)

        # Remove the previous yaws1
        self.current_yaw.remove()
        self.desired_yaw.remove()

        self.current_yaw = self.ax.arrow(
            0,
            0,
            -cur_x,
            -cur_y,
            head_width=0.1,
            head_length=0.1,
            fc="r",
            ec="r",
            linewidth=3,
        )
        self.desired_yaw = self.ax.arrow(
            0,
            0,
            -des_x,
            -des_y,
            head_width=0.1,
            head_length=0.1,
            fc="g",
            ec="g",
            linewidth=3,
        )

        self.cur_height_bar[0].set_height(cur_height / 10 if cur_height > 0 else 0)
        self.des_height_bar[0].set_height(des_height / 10 if des_height > 0 else 0)
        self.distance_bar[0].set_height(distance / 300 if distance > 0 else 0)

        self.current_yaw_value.set_text(
            f"Current: {(cur_yaw % (2*np.pi)) / np.pi * 180:.2f}˚"
        )
        self.current_yaw_value.set_color("r")  # Setting the text color to red

        self.desired_yaw_value.set_text(
            f"Desired: {(des_yaw % (2*np.pi)) / np.pi * 180:.2f}˚"
        )
        self.desired_yaw_value.set_color("g")  # Setting the text color to green

        self.desired_height_value.set_text(
            f"D{des_height if des_height > 0 else 0:.2f}"
        )
        self.desired_height_value.set_position((1.2, max(des_height / 10 - 0.9, -0.9)))
        self.desired_height_value.set_color("g")  # Setting the text color to green

        # Display height text
        self.current_height_value.set_text(
            f"C{cur_height if cur_height > 0 else 0:.2f}"
        )
        self.current_height_value.set_position((1.6, max(cur_height / 10 - 0.9, -0.8)))
        self.current_height_value.set_color("r")  # Setting the text color to red

        # Display distance value
        self.distance_value.set_text("{:06d}".format(int(distance)))
            #f"{distance if distance > 0 else 0:10.2f}"        )
        self.distance_value.set_position((2.2,-1.4))
        self.distance_value.set_color("b")  # Setting the text color to red

        self.battery_value.set_text(
            f"Battery: {battery:.2f} V"
        )
        if battery > LOW_BATTERY:
            self.battery_value.set_color("g")
        else:
            self.battery_value.set_color("r")



        plt.draw()

    def _draw_sliders(self):
        # Define the slider properties
        xoff=0.5
        slider_ax1 = plt.axes([xoff+0.2, 0.70, 0.20, 0.03], facecolor='white')
        slider_ax2 = plt.axes([xoff+0.2, 0.60, 0.2, 0.03], facecolor='lightgoldenrodyellow')
        slider_ax3 = plt.axes([xoff+0.2, 0.50, 0.2, 0.03], facecolor='lightgoldenrodyellow')
        slider_ax4 = plt.axes([xoff+0.2, 0.40, 0.2, 0.03], facecolor='lightgoldenrodyellow')
        slider_ax5 = plt.axes([xoff+0.2, 0.30, 0.2, 0.03], facecolor='lightgoldenrodyellow')




        # Define the slider variables and ranges
        self.force_x_slider = Slider(slider_ax1, 'Force x', 0, 1, valinit=0)
        self.z_level_slider = Slider(slider_ax2, 'z level', 1, 3, valinit=1)
        self.time_to_rotate_slider = Slider(slider_ax3, 'Time to Rotate', 0, 10, valinit=0)
        self.angle_slider = Slider(slider_ax4, 'Angle', -90, 90, valinit=0)
        self.zigzag_slider = Slider(slider_ax5, 'Zig Zag', 0, 15, valinit=0)

        self.force_x_slider.label.set_color('white')
        self.z_level_slider.label.set_color('white')
        self.time_to_rotate_slider.label.set_color('white')
        self.angle_slider.label.set_color('white')
        self.zigzag_slider.label.set_color('white')

        # self.force_x_slider.val.set_color('white')
        self.z_level_slider.label.set_color('white')
        self.time_to_rotate_slider.label.set_color('white')
        self.angle_slider.label.set_color('white')
        self.zigzag_slider.label.set_color('white')


        def update_sliders(val):
            force_x = self.force_x_slider.val
            z_level = self.z_level_slider.val
            time_to_rotate = self.time_to_rotate_slider.val
            angle = self.angle_slider.val
            zigzag = self.zigzag_slider.val
            # Connect the sliders to the update function

            # Add your code here to use the slider values
            print(
                f"Force x: {force_x}, z level: {z_level}, Time to Rotate: {time_to_rotate}, Angle: {angle}, Zig Zag: {zigzag}")

        self.force_x_slider.on_changed(update_sliders)
        self.z_level_slider.on_changed(update_sliders)
        self.time_to_rotate_slider.on_changed(update_sliders)
        self.angle_slider.on_changed(update_sliders)
        self.zigzag_slider.on_changed(update_sliders)


    def on_btn_reconnect_click(self, event):
        self.robConfig.initialize_system()
        print("Restart")

    def on_btn_flags_click(self, event):
        self.update_config_flags()

        self.robConfig.send_flags(read_file=False)
        print("Send Flags")

    def on_toggle_click(self, label):
        print(f'Toggle {label} clicked.')
        self.toggle_power = self.toggle.get_status()[0]

    def on_toggle_joy_click(self, label):
        print(f'Toggle {label} clicked.')
        self.toggle_joy = self.toggle2.get_status()[0]

    def on_radio_click(self, label):
        print(f'Radio button {label} selected.')
        # You can add logic here to handle radio button selection

    def sleep(self, delay=0.05):
        """
        Wait using plt
        :param delay: time to wait
        """
        if self.enable_gui:
            plt.pause(delay)
        else:
            time.sleep(delay)

    def update_config_flags(self):
        force_x = self.force_x_slider.val
        z_level = self.z_level_slider.val
        time_to_rotate = self.time_to_rotate_slider.val
        angle = self.angle_slider.val
        zigzag = self.zigzag_slider.val

        deterministic_walk = self.robConfig.get_config(ROBOT_JASON)['deterministic_walk']

        # [deterministic_walk["SWITCH_TIME"],
        # deterministic_walk["NUM_ZIGS"],
        # deterministic_walk["Z_LEVEL"],
        # deterministic_walk["TIME_ROTATE"],
        # deterministic_walk["ANGLE_THRESH"],
        # deterministic_walk["STEP_ZIG_ZAG"],
        # deterministic_walk["FORWARD_FORCE"]] = [10, 5, z_level, time_to_rotate, angle, zigzag, force_x]

        deterministic_walk["FORWARD_FORCE"] = force_x
        deterministic_walk["STEP_ZIG_ZAG"] = zigzag
        deterministic_walk["SWITCH_TIME"] = time_to_rotate * 1000



if __name__ == "__main__":
    mygui1 = SensorGUI(True)
    # mygui2 = SensorGUI(True)

    # Test plotting with increasing numbers
    for i in range(100):
        # mygui1.update_interface(i*2*pi/100, pi*random()/6, i*0.2, 0, i,0)
        mygui1.update_interface(i * 2 * pi / 100, pi * random() / 6, i * 0.2, 0, i,5*(1-1/(100-i)))
        mygui1.sleep()

    plt.ioff()
    plt.show()
