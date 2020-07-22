from pymeasure.instruments.arduino import ArduoinoGRBL
from pymeasure.adapters import VISAAdapter

import visa
import re
import time

rm = visa.ResourceManager()
VISASer = rm.open_resource('ASRL/dev/ttyACM0::INSTR', baud_rate=115200)
#adapter = VISAAdapter(VISASer)
stage = ArduoinoGRBL(VISASer)

stage.StepEnableInvert = 1
stage.StepIdleDelay = 255
stage.setconfig()




""" Test incremental Move
"""
posValue = 20
velValue = 1
stage.ask("G91 G0 Z%d F%d" % (posValue, velValue))



