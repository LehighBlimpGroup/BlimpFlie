from keyboardHandler import KeyboardHandler
from ESPNOW import ESPNOWControl
from robotConfig import RobotConfig
#from simpleGUI import SimpleGUI
import time
import math

#ESPNOW PARAMS
ESP_VERBOSE = False #True
PORT = "/dev/cu.usbmodem101"
LIST_OF_MAC_ADDRESS = [
    "34:85:18:91:BC:94",
    "34:85:18:91:BE:34",
    "48:27:E2:E6:EC:CC", #2 Sensor test drone
    "48:27:E2:E6:E4:0C", #3 Big diego drone first
    "48:27:E2:E6:DF:A0", #4 KKL Nicla drone
    "48:27:E2:E6:ED:24", #5 bingxu
    "48:27:E2:E6:DE:3C", #6
    "DC:54:75:D7:F7:FC", #7 hanqing
    "48:27:E2:E6:E6:44", #8 kim
    "34:85:18:91:24:F0", #9
    "34:85:18:91:20:a8", #10 Leo
]

MASTER_MAC = "34:85:18:91:BC:94" #address of the transiever

SLAVE_INDEX = 2 #-1 means broadcast

BRODCAST_CHANNEL = 1 # SLAVE_INDEX will override this value if SLAVE_INDEX is not -1


joyhandler = KeyboardHandler()
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS, ESP_VERBOSE)
robConfig = RobotConfig(esp_now, "robot_configs.json")

#gui = SimpleGUI()
#set configs for all slave indexes that you want to use 
#bicopter basic contains configs for a robot with no feedback
robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")
#robConfig.sendSetupFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")

robConfig.startBNO(BRODCAST_CHANNEL, SLAVE_INDEX)
# robConfig.startBaro(BRODCAST_CHANNEL, SLAVE_INDEX)
robConfig.startTranseiver(BRODCAST_CHANNEL, SLAVE_INDEX, MASTER_MAC)

# Nicla vision control
des_yaw = 0
des_height = 0
control_yaw = 0
control_height = 0
Ky1 = 1
Ky2 = 1
Kh1 = 1
Kh2 = 1
gamma1 = 0.8
gamma2 = 0.8
max_x = 240
max_y = 160
x, y, w, h = -1, -1, -1, -1

y = False
try:
    time_prev = time.time()
    while True:
        # under joystick control
        outputs, y = joyhandler.get_outputs()
        if y:
            break
        fx = outputs[1]
        #values from the Nicla
        feedback = esp_now.getFeedback(1) 
        _height = feedback[0]
        _yaw = feedback[1]
        _x = feedback[2] * 1000
        _y = feedback[3] * 1000
        _w = feedback[4] * 1000
        _h = feedback[5] * 1000



        #If the Nicla updates the desired yaw and height
        if x != _x or y != _y or w != _w or h != _h:
            x, y, w, h = _x, _y, _w, _h
            des_yaw = des_yaw*gamma1 - (((x - max_x/2)/max_x)*math.pi/2)*(1-gamma1)
            control_yaw = _yaw
            des_height = des_height*gamma2 + ((y - max_y/2)/max_y)*(1-gamma2)
            control_height = _height

        dt = time.time() - time_prev
        control_yaw += Ky1 * des_yaw * dt
        des_yaw -= Ky2 * des_yaw * dt
        control_height += Kh1 * des_height * dt
        des_height -= Kh2 * des_height * dt

        # Update the gui
        #gui.update_interface(control_yaw, des_yaw+control_yaw, control_height, des_height)

        # Print for debugging
        print("Yaw", _yaw, "control_yaw", round(control_yaw*180/3.14,2), "des_yaw", round(des_yaw*180/3.14,2))
        # print("control_height", control_height)
        # print("des_height", des_height)

        # outputs[]
        esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
        time_prev = time.time()
        time.sleep(0.02)
except Exception as e:
    print(e)
    if e == KeyboardInterrupt:
        print("Loop terminated by user.")
    else:
        print("Loop terminated by error.")
esp_now.send([0] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
esp_now.close()