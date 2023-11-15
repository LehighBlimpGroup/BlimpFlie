


ROBOT_JASON = "bicopterbasic"

#ESPNOW PARAMS
# MASTER_MAC = "34:85:18:91:C7:80" #address of the transceiver
ESP_VERBOSE = True
PORT = "/dev/tty.usbmodem14301" #serial port for the transiever


LIST_OF_MAC_ADDRESS = [
    # "34:85:18:91:BC:94",
    # "34:85:18:91:BE:34",
    # "48:27:E2:E6:EC:CC", #2 Sensor test drone
    # "48:27:E2:E6:E4:0C", #3 Big diego drone first
    # "48:27:E2:E6:DF:A0", #4 KKL Nicla drone
    # "48:27:E2:E6:ED:24", #5 bingxu
    # "48:27:E2:E6:DE:3C", #6
    # "DC:54:75:D7:F7:FC", #7 hanqing
    # "48:27:E2:E6:E6:44", #8 kim
    # "34:85:18:91:24:F0", #9
    # "48:27:E2:E6:E4:0C", #10 Big Wall
    # "48:27:E2:E6:E1:00",  # 11 david
    # "34:85:18:8F:36:B0" # Diego
    # "48:27:E2:E6:E6:44", #12 spinning blimp 1
    "34:85:18:91:B7:4C" #13 spinning blimp 2
    # "34:85:18:91:38:60", #14 spinning blimp 3
    # "34:85:18:AB:EF:E8", #15 spinning blimp 4
    # "34:85:18:AC:05:28", #16 spinning blimp 5
    # "34:85:18:91:20:A8", #small blended bicopter
    # "48:27:E2:E6:E6:D4", # jiawei's robot
    # "DC:54:75:D7:B3:E8" # attacker bicopter
]
MASTER_MAC = "48:27:E2:E6:E4:0C"
BRODCAST_CHANNEL = 1 # SLAVE_INDEX will override this value if SLAVE_INDEX is not -1


JOYSTICK_YAW_MODE = 1
GUI_ENABLED = True


MIN_Z = 0
MAX_Z = 50