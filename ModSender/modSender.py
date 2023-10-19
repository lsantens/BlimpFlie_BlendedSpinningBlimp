from joystickHandler import JoystickHandler
from ESPNOW import ESPNOWControl
from robotConfig import RobotConfig
import time

#ESPNOW PARAMS
# PORT = "COM5"
PORT = '/dev/ttyACM0'
LIST_OF_MAC_ADDRESS = [
    "34:85:18:91:20:a8",
]
SLAVE_INDEX = 0 #-1 means broadcast


BRODCAST_CHANNEL = 1 # SLAVE_INDEX will override this value if SLAVE_INDEX is not -1


joyhandler = JoystickHandler()
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS)
# robConfig = RobotConfig(esp_now, "ModSender\\robot_configs.json")
robConfig = RobotConfig(esp_now, "/home/jiawei/Desktop/SpinningControl_arduino_Moved/BlimpFlie-main Spin/ModSender/robot_configs.json")

#set configs for all slave indexes that you want to use 
#bicopter basic contains configs for a robot with no feedback

# function for switching between spinning blimp and bicopter gains
outputs_spin, y_spin = joyhandler.get_outputs()
if outputs_spin[8] == 1:
    robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic_spinning")
else:
    robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")
#robConfig.sendSetupFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")

#robConfig.startBNO(BRODCAST_CHANNEL, SLAVE_INDEX)
robConfig.startBaro(BRODCAST_CHANNEL, SLAVE_INDEX)

y = False
try:
    while not y:

        outputs, y = joyhandler.get_outputs()
        esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
        #esp_now.send(outputs, BRODCAST_CHANNEL, 0)
        time.sleep(0.02)
except KeyboardInterrupt:
    print("Loop terminated by user.")
esp_now.send([0] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
esp_now.close()