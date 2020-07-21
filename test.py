from pymeasure.instruments.arduino import ArduoinoGRBL
from pymeasure.adapters import VISAAdapter

import visa
import re
import time

rm = visa.ResourceManager()
VISASer = rm.open_resource('ASRL/dev/ttyACM0::INSTR', baud_rate=115200)
#adapter = VISAAdapter(VISASer)
stage = ArduoinoGRBL(VISASer)

stage.getconfig()

""" Clear error if exists
"""
stage.write("$X")
a = stage.read()
while (a != "ok\r\n"):
    print(repr(a))
    a = stage.read()

""" Test incremental Move
"""
posValue = 20
velValue = 1
stage.write("G91 G0 Z%d F%d" % (posValue, velValue))
a = stage.read()
while (a != "ok\r\n"):
    print(repr(a))
    a = stage.read()

stage.write("?")
print(repr(stage.read()))
print(repr(stage.read()))
print(repr(stage.read()))
print(repr(stage.read()))

while (a != "ok\r\n"):
    print(repr(a))
    a = stage.read()

stage.y.StepsPermm = 250
