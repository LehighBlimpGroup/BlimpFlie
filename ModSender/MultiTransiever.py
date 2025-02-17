from autonomy.ZigZagWalk import ZigZagWalk
from autonomy.RandomWalk import RandomWalk
from parameters import *
from teleop.joystickHandler import JoystickHandler
from comm.ESPNOW import ESPNOWControl
from robot.robotConfig import RobotConfig
from gui.sensorGUI2 import SensorGUI2


# Override feedback params for multiple robots
YAW_SENSOR = True
Z_SENSOR = True

def sensorgui2_reconnect_callback(id):
    robotConfigs[id].initialize_system()
    print("Reconnecting to robot %d: " % robotConfigs[id].slave_index, robotConfigs[id].mac)

def sensorgui2_close_callback(id):
    print("Closing connection to robot %d: " % robotConfigs[id].slave_index, robotConfigs[id].mac)
    esp_now.send([21, 0,0,0,0,0,0,0,0,0,0,0,0], BRODCAST_CHANNEL, robotConfigs[id].slave_index)    

def sensorgui2_send_flag_callback(id, flags):
    print("Sending flags to robot %d: " % robotConfigs[id].slave_index, robotConfigs[id].mac)
    deterministic_walk = robotConfigs[id].get_config(ROBOT_JASON)['deterministic_walk']
    force_x, z_level, time_to_rotate, angle, zigzag = flags
    print("Flags: ", flags)
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
    robotConfigs[id].send_flags(read_file=False)

# Joystick
joyhandler = JoystickHandler(yaw_sensor=YAW_SENSOR)

# Communication
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS, ESP_VERBOSE)

# For each robot

# Load robot configuration
robotConfigs = [RobotConfig(esp_now, i, robot_mac) for i, robot_mac in enumerate(LIST_OF_MAC_ADDRESS)]
# GUIS
sensor_guis = [SensorGUI2(GUI_ENABLED, id, sensorgui2_reconnect_callback, sensorgui2_close_callback, sensorgui2_send_flag_callback) for id in range(len(robotConfigs))]

# Send flags to each robot
for robConfig in robotConfigs:
    # Set configs for all slave indexes that you want to use
    print("Connecting to robot %d: "%robConfig.slave_index, robConfig.mac)
    robConfig.initialize_system()


# Autonomous Behavior
behavior_robots = [ZigZagWalk() for _ in robotConfigs]
for robot_behavior in behavior_robots:
    robot_behavior.begin()

###### Communicate until Y button (Exit) is pressed #####
y_pressed = False
try:
    while not y_pressed:
        outputs, y_pressed, a_key_pressed = joyhandler.get_outputs(yaw_mode=JOYSTICK_YAW_MODE)  # get joystick input

        # For each robot
        for i, robotConfig in enumerate(robotConfigs):
            feedback = esp_now.getFeedback(i)  # get sensor data from robot

            #  # ------- Autonomous mode ----------
            # if a_key_pressed:
            #     des_fx, des_z, des_yaw = behavior_robots[i].execute(feedback)
            #     outputs[1] = des_fx  # Forward
            #     outputs[3] = des_z  # Z
            #     outputs[6] = des_yaw  # Yaw control

             # ------- Autonomous mode ----------
            if a_key_pressed:
                # outputs[6] = 0
                outputs[8] = 2 # send 2 for randomwalk
                outputs[9] = 1
                print(outputs)
                print("autonomous")
                # des_fx, des_z, des_yaw = behavior_robots[i].execute(feedback)
                # outputs[1] = des_fx  # Forward
                # outputs[3] = des_z  # Z
                # joyhandler.tz = des_yaw  # Yaw control
            else:
                outputs[8] = 0  # send 0 for normal flight
                outputs[9] = 0
                print(outputs)

            # Display sensors and output
            # sensor_guis[i].update_nicla_box(nicla[0], 160 - nicla[1], nicla[2], nicla[3], 240, 160)
            sensor_guis[i].update(feedback[1], outputs[6], feedback[0], outputs[3], feedback[3], feedback[2], True)  # display sensor data


            if sensor_guis[i].use_toggled_connect:

                # Send message to all robots
                esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, robotConfig.slave_index)  # send control command to robot
            else:
                esp_now.send([21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], BRODCAST_CHANNEL, robotConfig.slave_index)
        # time.sleep(0.02)
        sensor_guis[0].sleep(0.02)

except KeyboardInterrupt:
    print("Loop terminated by user.")

for robotConfig in robotConfigs:
    esp_now.send([21, 0,0,0,0,0,0,0,0,0,0,0,0], BRODCAST_CHANNEL, robotConfig.slave_index)
esp_now.close()
