from pymeasure.instruments.arduino import ArduoinoGRBL
from pymeasure.adapters import VISAAdapter

import visa
import re
import time

""" 
workingconfig:
10.0
255.0
0.0
0.0
1.0
0.0
0.0
1.0
0.0
0.0
0.0
0.0
0.0
0.0
0.0
25.0
500.0
250.0
1.0
1000.0
0.0
0.0
250.0
250.0
250.0
500.0
500.0
500.0
10.0
10.0
10.0
200.0
200.0
200.0
"""

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



