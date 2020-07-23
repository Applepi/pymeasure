from pymeasure.instruments.arduino import ArduoinoGRBL
from pymeasure.adapters import VISAAdapter

import visa
import re
import time

rm = visa.ResourceManager()
VISASer = rm.open_resource('ASRL/dev/tty.usbmodem14301::INSTR', baud_rate=115200)
#adapter = VISAAdapter(VISASer)
stage = ArduoinoGRBL(VISASer)

a = stage.ask("")
while a != "ok\r\n":
    print(a)
    a = stage.ask("")

stage.ask("$X")
stage.setconfig()
stage.StepEnableInvert = 1
stage.StepIdleDelay = 255
stage.x.StepsPermm = 1610
stage.setconfig()

#stage.get_stored_config()
stage.ask("$$")

""" Test incremental Move
"""
posValue = 1
velValue = 1
stage.ask("G91 G0 X%d F%d" % (posValue, velValue))



