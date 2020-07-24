from pymeasure.instruments.arduino import ArduoinoGRBL
from pymeasure.adapters import VISAAdapter

import visa
import re
import time

rm = visa.ResourceManager()
# This value is computer specific, change to USB port of the GRBL controller
VISASer = rm.open_resource('ASRL/dev/tty.usbmodem14301::INSTR', baud_rate=115200)
stage = ArduoinoGRBL(VISASer)

# Clear the alarm status, for some reason on OSX GRBL starts in alarm state
a = stage.ask("$X")
while a != "ok\r\n":
    print(a)
    a = stage.ask("")


""" This is a testbed for using the Horiba T64000 Ramand System
    
    Initialzation junk specific to our machine
"""

#stage.setconfig()
stage.StepEnableInvert = 1      # Required for Current Stepper Drivers
stage.DirectionPortInvert = 1   # To align direction of travel to conter value
stage.StepIdleDelay = 255       # Leave motors energized
stage.x.StepsPermm = 50         # Upper Grating
stage.y.StepsPermm = 50         # Lower Grating
stage.z.StepsPermm = 10         # Vertical Entrance Slit
stage.x.MaxRate = 1000          # Homing is slow needs work
stage.y.MaxRate = 1000          # NOTE: Y-Homing doesn't seem to work either
stage.z.MaxRate = 10            # NOTE: Z-Homing is currently broken, needs further investigation
stage.x.MaxTravel = 5000        # Not acutal max travel, needs to be fixed before using in experiments
stage.y.MaxTravel = 5000
stage.z.MaxTravel = 5000
stage.HomingFeed = 500          # Could be fastetr
stage.HomingPulloff = 80        # Needs to be changed if steps/mm value is changed


stage.setconfig()               # Save config value to 

stage.home()

