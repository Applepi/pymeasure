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

import re

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

    def __init__(self, axis, controller):
        self.axis = str(axis)
        self.controller = controller
        
        self.StepsPermm = 250.0
        """
        $100, $101 and $102 – [X,Y,Z] steps/mm

        Grbl needs to know how far each step will take the tool in reality. 
        To calculate steps/mm for an axis of your machine you need to know:
            The mm traveled per revolution of your stepper motor. 
            This is dependent on your belt drive gears or lead screw pitch.
            The full steps per revolution of your steppers (typically 200)
            The microsteps per step of your controller (typically 1, 2, 4, 8, or 16). 
            Tip: Using high microstep values (e.g., 16) can reduce your stepper motor 
            torque, so use the lowest that gives you the desired axis resolution and 
            comfortable running properties.
        The steps/mm can then be calculated like this: 

        steps_per_mm = (steps_per_revolution*microsteps)/mm_per_rev

        Compute this value for every axis and write these settings to Grbl.
        """

        self.MaxRate = 500.0
        """
        $110, $111 and $112 – [X,Y,Z] Max rate, mm/min

        This sets the maximum rate each axis can move. Whenever Grbl plans a move, 
        it checks whether or not the move causes any one of these individual axes 
        to exceed their max rate. If so, it'll slow down the motion to ensure none 
        of the axes exceed their max rate limits. This means that each axis has its 
        own independent speed, which is extremely useful for limiting the typically 
        slower Z-axis.

        The simplest way to determine these values is to test each axis one at a time 
        by slowly increasing max rate settings and moving it. For example, to test the 
        X-axis, send Grbl something like G0 X50 with enough travel distance so that the 
        axis accelerates to its max speed. You'll know you've hit the max rate threshold 
        when your steppers stall. It'll make a bit of noise, but shouldn't hurt your 
        motors. Enter a setting a 10-20% below this value, so you can account for wear, 
        friction, and the mass of your workpiece/tool. Then, repeat for your other axes.

        NOTE: This max rate setting also sets the G0 seek rates.
        WARNING: Too Fast moves can corrupt the Arduino firmware
        """
        
        self.MaxAcceleration = 10.0
        """
        $120, $121, $122 – [X,Y,Z] Acceleration, mm/sec^2

        This sets the axes acceleration parameters in mm/second/second. Simplistically, 
        a lower value makes Grbl ease slower into motion, while a higher value yields 
        tighter moves and reaches the desired feed rates much quicker. Much like the 
        max rate setting, each axis has its own acceleration value and are independent 
        of each other. This means that a multi-axis motion will only accelerate as 
        quickly as the lowest contributing axis can.

        Again, like the max rate setting, the simplest way to determine the values 
        for this setting is to individually test each axis with slowly increasing 
        values until the motor stalls. Then finalize your acceleration setting 
        with a value 10-20% below this absolute max value. This should account for 
        wear, friction, and mass inertia. We highly recommend that you dry test some 
        G-code programs with your new settings before committing to them. Sometimes 
        the loading on your machine is different when moving in all axes together.
        """

        self.MaxTravel = 200.0
        """$130, $131, $132 – [X,Y,Z] Max travel, mm

        This sets the maximum travel from end to end for each axis in mm. This is 
        only useful if you have soft limits (and homing) enabled, as this is only 
        used by Grbl's soft limit feature to check if you have exceeded your machine 
        limits with a motion command.
        """


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



class ArduoinoGRBL(Instrument):
    """ Represents the GRBL Controller
    and provides a high-level for interacting with the instrument.

    By default this instrument is constructed with x, y, z
    attributes that represent axes 'X','Y','Z'.
    """

    error = Instrument.measurement(
        "TE?",
        """ Reads an error code from the motion controller.
        """,
        cast=int
    )

    def __init__(self, resourceName, **kwargs):
        super(ArduoinoGRBL, self).__init__(
            resourceName, "ArduoinoGRBL Motion Controller", **kwargs
        )
        # Defines default axes, which can be overwritten
        self.x = Axis('X', self)
        self.y = Axis('Y', self)
        self.z = Axis('Z', self)

        """ 
        Documetation of settings from 
        https://github.com/gnea/grbl/wiki/Grbl-v1.1-Configuration 
        """

        self.StepPulse = 10
        """ 
        $0 – Step pulse, microseconds

        Stepper drivers are rated for a certain minimum step pulse length. 
        Check the data sheet or just try some numbers. 
        You want the shortest pulses the stepper drivers can reliably recognize. 
        If the pulses are too long, you might run into trouble when running 
        the system at very high feed and pulse rates, 
        because the step pulses can begin to overlap each other. 
        We recommend something around 10 microseconds, which is the default value.
        """

        self.StepIdleDelay = 25
        """ 
        $1 - Step idle delay, milliseconds

        Every time your steppers complete a motion and come to a stop, 
        Grbl will delay disabling the steppers by this value. 
        OR, you can always keep your axes enabled (powered so as to hold position) 
        by setting this value to the maximum 255 milliseconds. 
        Again, just to repeat, you can keep all axes always enabled by setting $1=255.
        """

        self.StepPortInver = 0
        """ 
        $2 – Step port invert, mask

        This setting inverts the step pulse signal. By default, 
        a step signal starts at normal-low and goes high upon a 
        step pulse event. After a step pulse time set by $0, the 
        pin resets to low, until the next step pulse event. When 
        inverted, the step pulse behavior switches from normal-high, 
        to low during the pulse, and back to high. Most users will 
        not need to use this setting, but this can be useful for 
        certain CNC-stepper drivers that have peculiar requirements. 
        For example, an artificial delay between the direction pin 
        and step pulse can be created by inverting the step pin.

        This invert mask setting is a value which stores the axes 
        to invert as bit flags. You really don't need to completely 
        understand how it works. You simply need to enter the settings 
        value for the axes you want to invert. For example, if you 
        want to invert the X and Z axes, you'd send $2=5 to Grbl and 
        the setting should now read $2=5 (step port invert mask:00000101). 
        """

        self.DirectionPortInvert = 0
        """ 
        $3 – Direction port invert, mask

        This setting inverts the direction signal for each axis. 
        By default, Grbl assumes that the axes move in a positive direction 
        when the direction pin signal is low, and a negative direction when 
        the pin is high. Often, axes don't move this way with some machines. 
        This setting will invert the direction pin signal for those axes that 
        move the opposite way. This invert mask setting works exactly like the 
        step port invert mask and stores which axes to invert as bit flags
        """

        self.StepEnableInvert = 0
        """ 
        $4 - Step enable invert, boolean

        By default, the stepper enable pin is high to disable and low to enable. 
        If your setup needs the opposite, just invert the stepper enable pin by 
        typing $4=1. Disable with $4=0. (May need a power cycle to load the change.) 
        """

        self.LimitPinInvert = 0
        """ 
        $5 - Limit pins invert, boolean

        By default, the limit pins are held normally-high with the Arduino's internal 
        pull-up resistor. When a limit pin is low, Grbl interprets this as triggered. 
        For the opposite behavior, just invert the limit pins by typing $5=1. Disable 
        with $5=0. You may need a power cycle to load the change.

        NOTE: For more advanced usage, the internal pull-up resistor on the limit pins 
        may be disabled in config.h.
        """

        self.ProbePinInvert = 0
        """  
        $6 - Probe pin invert, boolean

        By default, the probe pin is held normally-high with the Arduino's 
        internal pull-up resistor. When the probe pin is low, Grbl interprets 
        this as triggered. For the opposite behavior, just invert the probe 
        pin by typing $6=1. Disable with $6=0. You may need a power cycle to 
        load the change.
        """

        self.StatusReport = 2
        """
        $10 - Status report, mask

        This setting determines what Grbl real-time data it reports back 
        to the user when a '?' status report is sent. This data includes 
        current run state, real-time position, real-time feed rate, pin 
        states, current override values, buffer states, and the g-code 
        line number currently executing (if enabled through compile-time options).

        By default, the new report implementation in Grbl v1.1+ will include 
        just about everything in the standard status report. A lot of the 
        data is hidden and will appear only if it changes. This increases 
        efficiency dramatically over of the old report style and allows you 
        to get faster updates and still get more data about your machine. 
        The interface documentation outlines how it works and most of it 
        applies only to GUI developers or the curious.

        To keep things simple and consistent, Grbl v1.1 has only two reporting options. 
        These are primarily here just for users and developers to help set things up.

            Position type may be specified to show either machine position (MPos:) 
            or work position (WPos:), but no longer both at the same time. Enabling 
            work position is useful in certain scenarios when Grbl is being directly 
            interacted with through a serial terminal, but machine position reporting 
            should be used by default.
            Usage data of Grbl's planner and serial RX buffers may be enabled. 
            This shows the number of blocks or bytes available in the respective buffers. 
            This is generally used to helps determine how Grbl is performing when testing 
            out a streaming interface. This should be disabled by default.

        Use the table below enables and disable reporting options. Simply add the 
        values listed of what you'd like to enable, then save it by sending Grbl your 
        setting value. For example, the default report with machine position and no 
        buffer data reports setting is $10=1. If work position and buffer data are 
        desired, the setting will be $10=2.

        Report Type 	Value 	Description
        Position Type 	0 	    Enable WPos: Disable MPos:.
        Position Type 	1 	    Enable MPos:. Disable WPos:.
        Buffer Data 	2 	    Enabled Buf: field appears with planner and serial RX available buffer.
        """

        self.JunctionDeviation = 0.010
        """
        $11 - Junction deviation, mm

        Junction deviation is used by the acceleration manager to determine how fast 
        it can move through line segment junctions of a G-code program path. 
        For example, if the G-code path has a sharp 10 degree turn coming up and 
        the machine is moving at full speed, this setting helps determine how much 
        the machine needs to slow down to safely go through the corner without losing steps.
        """

        self.ArcTolernce = 0.002
        """
        $12 – Arc tolerance, mm

        Grbl renders G2/G3 circles, arcs, and helices by subdividing them into teeny 
        tiny lines, such that the arc tracing accuracy is never below this value. 
        You will probably never need to adjust this setting, since 0.002mm is well 
        below the accuracy of most all CNC machines. But if you find that your circles 
        are too crude or arc tracing is performing slowly, adjust this setting. 
        Lower values give higher precision but may lead to performance issues 
        by overloading Grbl with too many tiny lines. Alternately, higher values 
        traces to a lower precision, but can speed up arc performance since Grbl 
        has fewer lines to deal with.
        """

        self.ReportInches = 0
        """
        $13 - Report inches, boolean

        Grbl has a real-time positioning reporting feature to provide a user 
        feedback on where the machine is exactly at that time, as well as, 
        parameters for coordinate offsets and probing. By default, it is set 
        to report in mm, but by sending a $13=1 command, you send this boolean 
        flag to true and these reporting features will now report in inches. 
        $13=0 to set back to mm.
        """

        self.SoftLimits = 0
        """ 
        $20 - Soft limits, boolean

        Soft limits is a safety feature to help prevent your machine from traveling 
        too far and beyond the limits of travel, crashing or breaking something expensive. 
        It works by knowing the maximum travel limits for each axis and where Grbl 
        is in machine coordinates. Whenever a new G-code motion is sent to Grbl, 
        it checks whether or not you accidentally have exceeded your machine space. 
        If you do, Grbl will issue an immediate feed hold wherever it is, shutdown the 
        spindle and coolant, and then set the system alarm indicating the problem. 
        Machine position will be retained afterwards, since it's not due to an immediate 
        forced stop like hard limits.

        NOTE: Soft limits requires homing to be enabled and accurate axis maximum 
        travel settings, because Grbl needs to know where it is. $20=1 to enable, 
        and $20=0 to disable.
        """

        self.HardLimits = 0
        """

        $21 - Hard limits, boolean

        Hard limit work basically the same as soft limits, but use physical switches instead. 
        Basically you wire up some switches (mechanical, magnetic, or optical) near the end of 
        travel of each axes, or where ever you feel that there might be trouble if your 
        program moves too far to where it shouldn't. When the switch triggers, it will 
        immediately halt all motion, shutdown the coolant and spindle (if connected), 
        and go into alarm mode, which forces you to check your machine and reset everything.

        To use hard limits with Grbl, the limit pins are held high with an internal pull-up resistor, 
        so all you have to do is wire in a normally-open switch with the pin and ground and 
        enable hard limits with $21=1. (Disable with $21=0.) We strongly advise taking 
        electric interference prevention measures. If you want a limit for both ends of 
        travel of one axes, just wire in two switches in parallel with the pin and ground, 
        so if either one of them trips, it triggers the hard limit.

        Keep in mind, that a hard limit event is considered to be critical event, 
        where steppers immediately stop and will have likely have lost steps. 
        Grbl doesn't have any feedback on position, so it can't guarantee it has any 
        idea where it is. So, if a hard limit is triggered, Grbl will go into an infinite 
        loop ALARM mode, giving you a chance to check your machine and forcing you to reset Grbl. 
        Remember it's a purely a safety feature.
        """

        self.HomingCycle = 1
        """
        $22 - Homing cycle, boolean

        Ahh, homing. For those just initiated into CNC, the homing cycle is used to accurately 
        and precisely locate a known and consistent position on a machine every time you start 
        up your Grbl between sessions. In other words, you know exactly where you are at any 
        given time, every time. Say you start machining something or are about to start the next 
        step in a job and the power goes out, you re-start Grbl and Grbl has no idea where it is 
        due to steppers being open-loop control. You're left with the task of figuring out where 
        you are. If you have homing, you always have the machine zero reference point to locate from, 
        so all you have to do is run the homing cycle and resume where you left off.

        To set up the homing cycle for Grbl, you need to have limit switches in a fixed position 
        that won't get bumped or moved, or else your reference point gets messed up. 
        Usually they are setup in the farthest point in +x, +y, +z of each axes. 
        Wire your limit switches in with the limit pins, add a recommended RC-filter to help 
        reduce electrical noise, and enable homing. If you're curious, you can use your limit 
        switches for both hard limits AND homing. They play nice with each other.
        """

        self.HomingDirection = 0
        """
        $23 - Homing dir invert, mask

        By default, Grbl assumes your homing limit switches are in the positive direction, 
        first moving the z-axis positive, then the x-y axes positive before trying to precisely 
        locate machine zero by going back and forth slowly around the switch. If your machine 
        has a limit switch in the negative direction, the homing direction mask can invert 
        the axes' direction. It works just like the step port invert and direction port invert 
        masks, where all you have to do is send the value in the table to indicate what axes 
        you want to invert and search for in the opposite direction.
        """

        self.HomingFeed = 25.000
        """
        $24 - Homing feed, mm/min

        The homing cycle first searches for the limit switches at a higher seek rate, and after 
        it finds them, it moves at a slower feed rate to home into the precise location of machine zero. 
        Homing feed rate is that slower feed rate. Set this to whatever rate value that provides repeatable 
        and precise machine zero locating.
        """

        self.HomingSeek = 500.000
        """
        $25 - Homing seek, mm/min

        Homing seek rate is the homing cycle search rate, or the rate at which it first tries to find the 
        limit switches. Adjust to whatever rate gets to the limit switches in a short enough time without 
        crashing into your limit switches if they come in too fast.
        """

        self.HomingDebounce = 250
        """
        $26 - Homing debounce, milliseconds
        
        Whenever a switch triggers, some of them can have electrical/mechanical noise that actually 'bounce' 
        the signal high and low for a few milliseconds before settling in. To solve this, you need to debounce 
        the signal, either by hardware with some kind of signal conditioner or by software with a short 
        delay to let the signal finish bouncing. Grbl performs a short delay, only homing when locating 
        machine zero. Set this delay value to whatever your switch needs to get repeatable homing. 
        In most cases, 5-25 milliseconds is fine.
        """
        
        self.HomingPulloff = 1.000
        """
        $27 - Homing pull-off, mm

        To play nice with the hard limits feature, where homing can share the same limit switches, 
        the homing cycle will move off all of the limit switches by this pull-off travel after it completes. 
        In other words, it helps to prevent accidental triggering of the hard limit after a homing cycle. 
        Make sure this value is large enough to clear the limit switch. If not, Grbl will throw an alarm 
        error for failing to clear it.
        """

        self.MaxSpindleSpeed = 1000
        """
        $30 - Max spindle speed, RPM

        This sets the spindle speed for the maximum 5V PWM pin output. For example, if you want to set 
        10000rpm at 5V, program $30=10000. For 255rpm at 5V, program $30=255. If a program tries to set 
        a higher spindle RPM greater than the $30 max spindle speed, Grbl will just output the max 5V, 
        since it can't go any faster. By default, Grbl linearly relates the max-min RPMs to 5V-0.02V 
        PWM pin output in 255 equally spaced increments. When the PWM pin reads 0V, this indicates 
        spindle disabled. Note that there are additional configuration options are available in config.h 
        to tweak how this operates.
        """

        self.MinSpindleSpeed = 0
        """
        $31 - Min spindle speed, RPM

        This sets the spindle speed for the minimum 0.02V PWM pin output (0V is disabled). Lower RPM values 
        are accepted by Grbl but the PWM output will not go below 0.02V, except when RPM is zero. If zero, 
        the spindle is disabled and PWM output is 0V.
        """

        self.LaserMode = 0
        """
        $32 - Laser mode, boolean

        When enabled, Grbl will move continuously through consecutive G1, G2, or G3 motion commands when 
        programmed with a S spindle speed (laser power). The spindle PWM pin will be updated instantaneously 
        through each motion without stopping. Please read the GRBL laser documentation and your laser device 
        documentation prior to using this mode. Lasers are very dangerous. They can instantly damage your 
        vision permanantly and cause fires. Grbl does not assume any responsibility for any issues the 
        firmware may cause, as defined by its GPL license.

        When disabled, Grbl will operate as it always has, stopping motion with every S spindle speed command. 
        This is the default operation of a milling machine to allow a pause to let the spindle change speeds.
        """

    def verified_write(self, command):
        a = self.ask(command)
        sleep(0.1)
        if a != "ok\r\n":
            print(a)
            print("Error was encountered while performing %s" % command)


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

    def get_stored_config(self):
        """ Returns configuration values to terminal
        """
        self.write("$$")
        a = self.read()
        while (a != "ok\r\n"):
            a = float(re.findall(r'=(\w+)',a)[0])
            print(repr(a))
            a = self.read()
    
    def global_config(self, command, value='\0'):
        """ Allows configuration of GRBL values as defined here:
        https://github.com/gnea/grbl/wiki/Grbl-v1.1-Configuration
        """
        if value == '\0':
            self.ask("$%d" % (command))
        else:
            self.ask("$%d=%f" % (command, value))

    def setconfig(self):
        """
        Sets current configuration to the current stored
        """
        self.verified_write("$0=%d" % self.StepPulse)
        self.verified_write("$1=%d" % self.StepIdleDelay)
        self.verified_write("$2=%d" % self.StepPortInver)
        self.verified_write("$3=%d" % self.DirectionPortInvert)
        self.verified_write("$4=%d" % self.StepEnableInvert)
        self.verified_write("$5=%d" % self.LimitPinInvert)
        self.verified_write("$6=%d" % self.ProbePinInvert)
        self.verified_write("$10=%d" % self.StatusReport)
        self.verified_write("$11=%f" % self.JunctionDeviation)
        self.verified_write("$12=%f" % self.ArcTolernce)
        self.verified_write("$13=%d" % self.ReportInches)
        self.verified_write("$20=%d" % self.SoftLimits)
        self.verified_write("$21=%d" % self.HardLimits)
        self.verified_write("$22=%d" % self.HomingCycle)
        self.verified_write("$23=%d" % self.HomingDirection)
        self.verified_write("$24=%f" % self.HomingFeed)
        self.verified_write("$25=%f" % self.HomingSeek)
        self.verified_write("$26=%d" % self.HomingDebounce)
        self.verified_write("$27=%f" % self.HomingPulloff)
        self.verified_write("$30=%f" % self.MaxSpindleSpeed)
        self.verified_write("$31=%f" % self.MinSpindleSpeed)
        self.verified_write("$32=%d" % self.LaserMode)
        self.verified_write("$100=%f" % self.x.StepsPermm)
        self.verified_write("$101=%f" % self.y.StepsPermm)
        self.verified_write("$102=%f" % self.z.StepsPermm)
        self.verified_write("$110=%f" % self.x.MaxRate)
        self.verified_write("$111=%f" % self.y.MaxRate)
        self.verified_write("$112=%f" % self.z.MaxRate)
        self.verified_write("$120=%f" % self.x.MaxAcceleration)
        self.verified_write("$121=%f" % self.y.MaxAcceleration)
        self.verified_write("$122=%f" % self.z.MaxAcceleration)
        self.verified_write("$130=%f" % self.x.MaxTravel)
        self.verified_write("$131=%f" % self.y.MaxTravel)
        self.verified_write("$132=%f" % self.z.MaxTravel)
        