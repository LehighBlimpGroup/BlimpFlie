

ROBOT_CONFIG_FILE = "./robot_configs.json"

#ESPNOW PARAMS
# MASTER_MAC = "34:85:18:91:C7:80" #address of the transceiver
ESP_VERBOSE = True
PORT = "COM12" #serial port for the transiever

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
    "48:27:E2:E6:E1:00", #11 david
]
MASTER_MAC = "C0:49:EF:EB:FE:34"
SLAVE_INDEX = 0 #-1 means broadcast
BRODCAST_CHANNEL = 1 # SLAVE_INDEX will override this value if SLAVE_INDEX is not -1

YAW_SENSOR = True
Z_SENSOR = False
GUI_ENABLED = True