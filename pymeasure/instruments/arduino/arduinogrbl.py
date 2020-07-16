#
# This file is part of the PyMeasure package.
#
# Copyright (c) 2013-2020 PyMeasure Developers
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

from time import sleep

from pymeasure.instruments import Instrument
from pymeasure.instruments.validators import strict_discrete_set


class AlarmError(Exception):
    """ Raised when GRBL is entering an alarm state """

    MESSAGES = {
        '1': 'Hard limit triggered. Machine position is likely lost due to sudden and immediate halt. Re-homing is highly recommended.',
        '2': 'G-code motion target exceeds machine travel. Machine position safely retained. Alarm may be unlocked.',
        '3': 'Reset while in motion. Grbl cannot guarantee position. Lost steps are likely. Re-homing is highly recommended.',
        '4': 'Probe fail. The probe is not in the expected initial state before starting probe cycle, where G38.2 and G38.3 is not triggered and G38.4 and G38.5 is triggered.',
        '5': 'Probe fail. Probe did not contact the workpiece within the programmed travel for G38.2 and G38.4.',
        '6': 'Homing fail. Reset during active homing cycle.',
        '7': 'Homing fail. Safety door was opened during active homing cycle.',
        '8': 'Homing fail. Cycle failed to clear limit switch when pulling off. Try increasing pull-off setting or check wiring.',
        '9': 'Homing fail. Could not find limit switch within search distance. Defined as 1.5 * max_travel on search and 5 * pulloff on locate phases.',
    }

    def __init__(self, code):
        self.error = str(code)[1:]
        self.message = self.MESSAGES[self.error]

    def __str__(self):
        return "GRBL reported an ALARM: %s" % (
            self.message)

class GeneralError(Exception):
    """ Raised when the GRBL has a general error.
    """

    MESSAGES = {
        '1': 'G-code words consist of a letter and a value. Letter was not found.',
        '2': 'Numeric value format is not valid or missing an expected value.',
        '3': 'Grbl \'$\' system command was not recognized or supported.',
        '4': 'Negative value received for an expected positive value.',
        '5': 'Homing cycle is not enabled via settings.',
        '6': 'Minimum step pulse time must be greater than 3usec',
        '7': 'EEPROM read failed. Reset and restored to default values.',
        '8': 'Grbl \'$\' command cannot be used unless Grbl is IDLE. Ensures smooth operation during a job.',
        '9': 'G-code locked out during alarm or jog state',
        '10': 'Soft limits cannot be enabled without homing also enabled.',
        '11': 'Max characters per line exceeded. Line was not processed and executed.',
        '12': '(Compile Option) Grbl \'$\' setting value exceeds the maximum step rate supported.',
        '13': 'Safety door detected as opened and door state initiated.',
        '14': '(Grbl-Mega Only) Build info or startup line exceeded EEPROM line length limit.',
        '15': 'Jog target exceeds machine travel. Command ignored.',
        '16': 'Jog command with no \'=\' or contains prohibited g-code.',
        '17': 'Laser mode requires PWM output.',
        '20': 'Unsupported or invalid g-code command found in block.',
        '21': 'More than one g-code command from same modal group found in block.',
        '22': 'Feed rate has not yet been set or is undefined.',
        '23': 'G-code command in block requires an integer value.',
        '24': 'Two G-code commands that both require the use of the XYZ axis words were detected in the block.',
        '25': 'A G-code word was repeated in the block.',
        '26': 'A G-code command implicitly or explicitly requires XYZ axis words in the block, but none were detected.',
        '27': 'N line number value is not within the valid range of 1 - 9,999,999.',
        '28': 'A G-code command was sent, but is missing some required P or L value words in the line.',
        '29': 'Grbl supports six work coordinate systems G54-G59. G59.1, G59.2, and G59.3 are not supported.',
        '30': 'The G53 G-code command requires either a G0 seek or G1 feed motion mode to be active. A different motion was active.',
        '31': 'There are unused axis words in the block and G80 motion mode cancel is active.',
        '32': 'A G2 or G3 arc was commanded but there are no XYZ axis words in the selected plane to trace the arc.',
        '33': 'The motion command has an invalid target. G2, G3, and G38.2 generates this error, if the arc is impossible to generate or if the probe target is the current position.',
        '34': 'A G2 or G3 arc, traced with the radius definition, had a mathematical error when computing the arc geometry. Try either breaking up the arc into semi-circles or quadrants, or redefine them with the arc offset definition.',
        '35': 'A G2 or G3 arc, traced with the offset definition, is missing the IJK offset word in the selected plane to trace the arc.',
        '36': 'There are unused, leftover G-code words that aren\'t used by any command in the block.',
        '37': 'The G43.1 dynamic tool length offset command cannot apply an offset to an axis other than its configured axis. The Grbl default axis is the Z-axis.',
        '38': 'Tool number greater than max supported value.',
    }

    def __init__(self, code):
        self.error = str(code)
        self.message = self.MESSAGES[self.error]

    def __str__(self):
        return "GRBL reported the error: %s" % (
            self.message)
# Not yet implmented
class Axis(object):
    """ Represents an axis of the GRBL,
    which can have independent parameters from the other axes.
    """

    position = Instrument.control(
        "TP", "PA%g",
        """ A floating point property that controls the position
        of the axis. The units are defined based on the actuator.
        Use the :meth:`~.wait_for_stop` method to ensure the position
        is stable.
        """
    )
    enabled = Instrument.measurement(
        "MO?",
        """ Returns a boolean value that is True if the motion for
        this axis is enabled.
        """,
        cast=bool
    )
    left_limit = Instrument.control(
        "SL?", "SL%g",
        """ A floating point property that controls the left software
        limit of the axis. """
    )
    right_limit = Instrument.control(
        "SR?", "SR%g",
        """ A floating point property that controls the right software
        limit of the axis. """
    )
    units = Instrument.control(
        "SN?", "SN%d",
        """ A string property that controls the displacement units of the
        axis, which can take values of: enconder count, motor step, millimeter,
        micrometer, inches, milli-inches, micro-inches, degree, gradient, radian,
        milliradian, and microradian.
        """,
        validator=strict_discrete_set,
        values={
            'encoder count':0, 'motor step':1, 'millimeter':2,
            'micrometer':3, 'inches':4, 'milli-inches':5,
            'micro-inches':6, 'degree':7, 'gradient':8,
            'radian':9, 'milliradian':10, 'microradian':11
        },
        map_values=True
    )
    motion_done = Instrument.measurement(
        "MD?",
        """ Returns a boolean that is True if the motion is finished.
        """,
        cast=bool
    )

    def __init__(self, axis, controller):
        self.axis = str(axis)
        self.controller = controller

    def ask(self, command):
        command = self.axis + command
        return self.controller.ask(command)

    def write(self, command):
        command = self.axis + command
        self.controller.write(command)

    def values(self, command, **kwargs):
        command = self.axis + command
        return self.controller.values(command, **kwargs)

    def enable(self):
        """ Enables motion for the axis. """
        self.write("MO")

    def disable(self):
        """ Disables motion for the axis. """
        self.write("MF")

    def home(self, type=1):
        """ Drives the axis to the home position, which may be the negative
        hardware limit for some actuators (e.g. LTA-HS).
        type can take integer values from 0 to 6.
        """
        home_type = strict_discrete_set(type, [0,1,2,3,4,5,6])
        self.write("OR%d" % home_type)

    def define_position(self, position):
        """ Overwrites the value of the current position with the given
        value. """
        self.write("DH%g" % position)

    def zero(self):
        """ Resets the axis position to be zero at the current poisiton.
        """
        self.write("DH")

    def wait_for_stop(self, delay=0, interval=0.05):
        """ Blocks the program until the motion is completed. A further
        delay can be specified in seconds.
        """
        self.write("WS%d" % (delay*1e3))
        while not self.motion_done:
            sleep(interval)


class ArduoinoGRBL(Instrument):
    """ Represents the GRBL Controller
    and provides a high-level for interacting with the instrument.

    By default this instrument is constructed with x, y, z, and a
    attributes that represent axes 1, 2, 3, 4. Custom implementations
    can overwrite this depending on the avalible axes. Axes are controlled
    through an :class:`Axis <pymeasure.instruments.newport.ArduinoGRBL.Axis>`
    class.
    """

    error = Instrument.measurement(
        "TE?",
        """ Reads an error code from the motion controller.
        """,
        cast=int
    )

    def __init__(self, resourceName, **kwargs):
        super(ArduoinoGRBL, self).__init__(
            resourceName,
            "ArduoinoGRBL Motion Controller",
            **kwargs
        )
        # Defines default axes, which can be overwritten
        self.x = Axis('X', self)
        self.y = Axis('Y', self)
        self.z = Axis('Z', self)
        self.a = Axis('A', self)

    def clear_errors(self):
        """ Clears the error messages by checking until a 0 code is
        recived. """
        while self.error != 0:
            continue

    @property
    def errors(self):
        """ Returns a list of error Exceptions that can be later raised, or
        used to diagnose the situation.
        """
        errors = []

        code = self.error
        while code != 0:
            if code > 100:
                errors.append(AlarmError(code))
            else:
                errors.append(GeneralError(code))
            code = self.error
        return errors

    @property
    def axes(self):
        """ A list of the :class:`Axis <pymeasure.instruments.newport.esp300.Axis>`
        objects that are present. """
        axes = []
        directory = dir(self)
        for name in directory:
            if name == 'axes':
                continue # Skip this property
            try:
                item = getattr(self, name)
                if isinstance(item, Axis):
                    axes.append(item)
            except TypeError:
                continue
            except Exception as e:
                raise e
        return axes

    def enable(self):
        """ Enables all of the axes associated with this controller.
        """
        for axis in self.axes:
            axis.enable()

    def disable(self):
        """ Disables all of the axes associated with this controller.
        """
        for axis in self.axes:
            axis.disable()

    def shutdown(self):
        """ Shuts down the controller by disabling all of the axes.
        """
        self.disable()

    def getconfig(self):
        """ Returns configuration values to terminal
        """
    