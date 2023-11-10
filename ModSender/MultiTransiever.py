from autonomy.ZigZagWalk import DeterministicWalk
from autonomy.RandomWalk import RandomWalk
from parameters import *
from teleop.joystickHandler import JoystickHandler
from comm.ESPNOW import ESPNOWControl
from robot.robotConfig import RobotConfig
# from gui.visualizer import SensorGUI
from gui.simpleGUI import SimpleGUI 
import sys

# Override feedback params for multiple robots
YAW_SENSOR = True
Z_SENSOR = True

# User interface

# Joystick
joyhandler = JoystickHandler(yaw_sensor=YAW_SENSOR)

# Communication
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS, ESP_VERBOSE)

# For each robot

# Load robot configuration
robotConfigs = [RobotConfig(esp_now, i, robot_mac) for i, robot_mac in enumerate(LIST_OF_MAC_ADDRESS)]
# GUIS
enable_wall_sensor, enable_vision, enable_battry = False, False, False
sensor_guis = [SimpleGUI(enable_wall_sensor=enable_wall_sensor, enable_vision=enable_vision, enable_battry=enable_battry) for robConfig in robotConfigs]


# Send flags to each robot
for robConfig in robotConfigs:
    # Set configs for all slave indexes that you want to use
    print("Connecting to robot %d: "%robConfig.slave_index, robConfig.mac)
    robConfig.initialize_system()


# Autonomous Behavior
behavior_robots = [DeterministicWalk() for _ in robotConfigs]
for robot_behavior in behavior_robots:
    robot_behavior.begin()

###### Communicate until Y button (Exit) is pressed #####
y_pressed = False
try:
    while not y_pressed:
        outputs, y_pressed, a_key_pressed = joyhandler.get_outputs(yaw_mode=JOYSTICK_YAW_MODE)  # get joystick input
        outputs[8] = int(a_key_pressed)
        
        # For each robot
        for i, robotConfig in enumerate(robotConfigs):
            # feedback[0] : height
            # feedback[1] : yaw
            # feedback[2] : time of flight
            # feedback[3] : battery level
            # feedback[4] : motor output
            # feedback[5] : servo output
            feedback = esp_now.getFeedback(1)  # get sensor data from robot
            # nicla[0] : x
            # nicla[1] : y
            # nicla[2] : w
            # nicla[3] : h
            # nicla[4] : time of flight
            # nicla[5] : battery level
            nicla = esp_now.getFeedback(2)  # get sensor data from robot
             # ------- Autonomous mode ----------
            # if a_key_pressed:
            #     des_fx, des_z, des_yaw = behavior_robots[i].execute(feedback)
            #     outputs[1] = des_fx  # Forward
            #     outputs[3] = des_z  # Z
            #     joyhandler.tz = des_yaw  # Yaw control

            # Display sensors and output
            # sensor_guis[i].update_nicla_box(nicla[0], 160 - nicla[1], nicla[2], nicla[3], 240, 160)
            # sensor_guis[i].update_interface(feedback[1], outputs[6], feedback[0], outputs[3], feedback[1])  # display sensor data

            roi = [nicla[0], nicla[1], nicla[2], nicla[3]]
            flag = sensor_guis[i].update_interface(cur_yaw=feedback[1], des_yaw=outputs[6], cur_height=feedback[0], des_height=outputs[3], distance=0, roi=roi, batlevel=feedback[3], connection=True)
            if flag == 0:
                break
            if flag == -1:
                sys.exit()

            # Send message to all robots
            esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, robotConfig.slave_index)  # send control command to robot


        # time.sleep(0.02)
        sensor_guis[0].sleep(0.02)

except KeyboardInterrupt:
    print("Loop terminated by user.")

for robotConfig in robotConfigs:
    esp_now.send([21, 0,0,0,0,0,0,0,0,0,0,0,0], BRODCAST_CHANNEL, robotConfig.slave_index)
esp_now.close()
