from autonomy.RandomWalk import RandomWalk
from parameters import *
from teleop.joystickHandler import JoystickHandler
from comm.ESPNOW import ESPNOWControl
from robot.robotConfig import RobotConfig
from gui.visualizer import SensorGUI


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
robotConfigs = [RobotConfig(esp_now, "config/"+robot_mac+".json") for robot_mac in LIST_OF_MAC_ADDRESS]
# GUIS
sensor_guis = [SensorGUI(GUI_ENABLED) for robConfig in robotConfigs]

# Send flags to each robot
for robConfig in robotConfigs:
    # Set configs for all slave indexes that you want to use
    # Bicopter basic contains configs for a robot with no feedback
    active = robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, ROBOT_JASON)
    robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, ROBOT_JASON)  # Redundant sent.


    if YAW_SENSOR:
        robConfig.startBNO(BRODCAST_CHANNEL, SLAVE_INDEX)  # Configure IMU

    if Z_SENSOR:
        robConfig.startBaro(BRODCAST_CHANNEL, SLAVE_INDEX)  # Configure Barometer


    robConfig.startThrustRange(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")  # Motor specifications
    robConfig.startTranseiver(BRODCAST_CHANNEL, SLAVE_INDEX, MASTER_MAC)  # Start communication


# Autonomous Behavior
autonomous = RandomWalk()
autonomous.begin()

###### Communicate until Y button (Exit) is pressed #####
y_pressed = False
try:
    while not y_pressed:
        outputs, y_pressed = joyhandler.get_outputs()  # get joystick input
        # outputs = [0]*13
        feedback = esp_now.getFeedback(1)  # get sensor data from robot
        # print(feedback)

        # ------- Autonomous mode ----------
        a_key_pressed = joyhandler.a_state
        if a_key_pressed:
            des_fx, des_z, des_yaw = autonomous.execute(feedback)
            outputs[1] = des_fx  # Forward
            outputs[3] = des_z  # Z
            joyhandler.tz = des_yaw  # Yaw control

        # Display sensors and output
        sensor_gui.update_interface(feedback[3], outputs[6], feedback[0], outputs[3], feedback[5])  # display sensor data
        # Communicate with robot
        esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)  # send control command to robot


        # time.sleep(0.02)
        sensor_gui.sleep(0.02)

except KeyboardInterrupt:
    print("Loop terminated by user.")
esp_now.send([21, 0,0,0,0,0,0,0,0,0,0,0,0], BRODCAST_CHANNEL, SLAVE_INDEX)
esp_now.close()
