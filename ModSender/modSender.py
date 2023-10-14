from joystickHandler import JoystickHandler
from ESPNOW import ESPNOWControl
from robotConfig import RobotConfig
import time

#ESPNOW PARAMS
PORT = "COM5"
LIST_OF_MAC_ADDRESS = [
    "34:85:18:91:BC:94",
    "34:85:18:91:BE:34",
]
SLAVE_INDEX = 1 #-1 means broadcast


BRODCAST_CHANNEL = 1 # SLAVE_INDEX will override this value if SLAVE_INDEX is not -1


joyhandler = JoystickHandler()
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS)
robConfig = RobotConfig(esp_now, "ModSender\\robot_configs.json")

#set configs for all slave indexes that you want to use 
#bicopter basic contains configs for a robot with no feedback
robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")

y = False
try:
    while not y:

        outputs, y = joyhandler.get_outputs()
        
        esp_now.send(outputs, BRODCAST_CHANNEL, SLAVE_INDEX)
        #esp_now.send(outputs, BRODCAST_CHANNEL, 0)
        time.sleep(0.02)
except KeyboardInterrupt:
    print("Loop terminated by user.")
esp_now.close()