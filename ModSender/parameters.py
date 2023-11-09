


ROBOT_JASON = "bicopterbasic"

#ESPNOW PARAMS
# MASTER_MAC = "34:85:18:91:C7:80" #address of the transceiver
ESP_VERBOSE = True
PORT = "/dev/cu.usbmodem1301" #serial port for the transiever


LIST_OF_MAC_ADDRESS = [
#     # "DC:54:75:D8:01:04", #0 KkL Sender
#     # "34:85:18:91:B5:B4", #1 KKL Back-up sender
    # "48:27:E2:E6:EC:CC", #2 Sensor test drone
#     # "48:27:E2:E6:E4:0C", #3 Big diego drone first
      "48:27:E2:E6:DF:A0" #4 KKL Nicla drone
#     # "48:27:E2:E6:ED:24", #5 bingxu
#     # "48:27:E2:E6:DE:3C", #6
#     # "DC:54:75:D7:F7:FC", #7 hanqing
#     # "48:27:E2:E6:E6:44", #8 kim
#     # "34:85:18:91:24:F0", #9
#     # "48:27:E2:E6:E4:0C", #10 Big Wall
    # "48:27:E2:E6:E1:00", #11 david
    # "34:85:18:8F:36:B0", # Diego
    # "48:27:E2:E6:ED:24", # Bella
    # "34:85:18:91:BC:94" # Joy
]

MASTER_MAC = "34:85:18:91:B5:B4"#address of the transceiver
BRODCAST_CHANNEL = 1             # SLAVE_INDEX will override this value if SLAVE_INDEX is not -

GUI_ENABLED = True

MIN_Z = 0
MAX_Z = 50