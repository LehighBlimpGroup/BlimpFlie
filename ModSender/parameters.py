


ROBOT_JASON = "bicopterbasic"

#ESPNOW PARAMS
# MASTER_MAC = "34:85:18:91:C7:80" #address of the transceiver
ESP_VERBOSE = True
PORT = "COM16" #serial port for the transiever


LIST_OF_MAC_ADDRESS = [
    # "48:27:E2:E6:EC:CC", # Edward - 1
    # "48:27:E2:E6:E6:D4", # Jiawei - 2
    # "48:27:E2:E6:ED:24", # Bella - 3
    # "34:85:18:8D:8B:38", # KKL2 - 4
    # "34:85:18:8F:36:B0", # Takeru - 5
    # "48:27:E2:E6:DF:A0", #4 KKL1 - 6
    "34:85:18:91:BC:94", # Joy - 7
    # "34:85:18:AC:C2:6C" # KKL3 - 8

]

MASTER_MAC = "34:85:18:91:24:F0" #"C0:49:EF:E3:34:78"
BRODCAST_CHANNEL = 1 # SLAVE_INDEX will override this value if SLAVE_INDEX is not -1


JOYSTICK_YAW_MODE = 0
GUI_ENABLED = True


MIN_Z = 0
MAX_Z = 50