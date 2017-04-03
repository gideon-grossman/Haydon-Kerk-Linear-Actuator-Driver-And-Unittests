"""Module for IDEA Drive pcm4826e stepper motor controller.

This module is used to control the IDEA Drive pcm4826e stepper motor controller
, which Omniome is using to control a Haydon Kerk linear actuator. It
contains the Pcm4826e class on initialization.

The Pcm4826e class is intended to automatically find the correct serial port
 in instantiating the class, and to close without crashing (i.e. releasing
the serial port resources automatically).

Typically, the Pcm4826e class is integrated with a valve control class to make
a fluidics class that coordinates pumping and valve control.

To Do:
1.Check that the initialization procedure init_pump
    is necessary and sufficient.
2. It might be necessary to include the stop method
    in the shutdown and destructor methods

Example
-------
The following commands will initialize the motor,
advance the motor by 100,000 increments,then home the motor.

    >>> from omnifluidics.IDEA_actuator import Pcm4826e
    >>> pcm4826e_control = Pcm4826e()
    >>> pcm4826e_control.init_pump()
    >>> pcm4826e_control.plunger_move_by(100000)
    >>> pcm4826e_control.plunger_move_to(0)
    >>> del pcm4826e_control

Notes
-----
For more information read the 'IDEA Drive Communication Manual'
.. IDEA Drive Communication manual:
http://haydonkerk.com/Portals/0/IDEA%20Support/IDEA%202013%20Manuals/IDEA%20Drive%20Communication%20Manual.pdf
You can find this document in the resources folder.
"""

from __future__ import print_function
import time
import serial
from serial import SerialException
import sys
import glob
from configuration import FluidicsConfig
config = FluidicsConfig()
from cavro import list_serial_ports
import inspect
import time


class CheckMotorPluggedIn(Exception):
    def __init__(self, message):
        self.message = message

class Pcm4826e(object):
    """
    Class for controlling the Pcm4826e
    HaydonKerk IDEA stepper motor linear actuator controller

    Uses the ``serial.Serial`` class to create methods for controlling the
    Pcm4826e linear actuator controller. Automatically finds the serial ports
    and assumes the first one is the correct one. Upon deletion of the method
    calls ``Serial.close()`` to free up the serial port so python can exit
    without a system crash.

    Attributes
    ----------
    incrementsPerStep : int
        Increments per stepper motor step. Can be 1, 2, 4 8, 16, 32, or 64
    stepsPerMm : int
        Steps the stepper motor takes to move the linear actuator 1 mm.
        This value is a physical property ofthe linear actuator and cannot be changed.
    accelRate : int
        Rate at which the speed should rise from the Start Speed to the Run Speed.
        0, or 500 to 16777215 increments per second per second.
    decelRate : int
         Rate at which the speed should fall from the Run Speed to the Final Speed.
         0, or 500 to 16777215 increments per second per second.
    runCurrent : int
        The rms current, in milliamps for the move. Must be less than 600.
        0 to 3850 dependant on drive (490 for the pcm4826e model).
    holdCurrent : int
        The rms current, in milliamps, for after the move has completed. Must be less than 600.
        0 to 3850 dependant on drive (490 for the pcm4826e model)
    accelCurrent : int
        The rms current, in milliamps, for the acceleration portion of the move.
        0 to 3850 dependant on drive (490 for the pcm4826e model)
    decelCurrent : int
        The rms current, in milliamps, for the deceleration portion of the move.
        0 to 3850 dependant on drive (490 for the pcm4826e model)
    delay : int
        The time, in milliseconds, between the last step of a move and when the current is set to the hold current.
    stopEndSpeed : int
        The number of steps per second the motor should move when ending the move in the given step mode.
        0 or 50 to 7500 (increments per second). Must be less than run speed.
    stopDecelRate : int
        Rate at which the speed should fall from the current speed to the end speed.
        0 or 500 to 16777215 (increments per second per second).
    stopRunCurrent : int
        The rms current, in milliamps for the deceleration,
        if too long to use boosted decel current for the entire ramp.
        0 to 3850 dependant on drive (490 for the pcm4826e model).
    stopHoldCurrent : int
        The rms current, in milliamps, for after the move has completed.
         0 to 3850 dependant on drive (490 for the pcm4826e model).
    stopDecelCurrent : int
        The rms current, in milliamps, for the deceleration portion of the move
        0 to 3850 dependant on drive (490 for the pcm4826e model).
    stopDelay :  int
        The time, in milliseconds, between the last step of a move
        and when the current is set to the hold current for the stop command.
    serial : serial.Serial
        The serial object configured with ``baudrate=57600``, ``parity=
        serial.PARITY_NONE``, ``stopbits=serial.STOPBITS_ONE`` and ``bytesize=
        serial.EIGHTBITS``.
    has_pump : bool
        Keeps track of whether or not the linear actuator is connected.

    """
    def __init__(self, actuator_serial_device):
        """Connects to pump.

        Note
        ----
        The pump hardware box communicates with the serial port using USB power. Meaning the serial port may be
        recognized but the pump not powered.

        """
        self.incrementsPerStep = 64
        self.stepsPerMm = 100
        self.plunger_speed = int(config.MAX_SPEED_UL_S / config.DISTANCE_UL_PER_MM * 100 * 64)
        self.accelRate = 16777215
        self.decelRate = 16777215
        self.runCurrent = 245
        self.holdCurrent = 245#
        self.accelCurrent = 245
        self.decelCurrent = 245
        self.delay = 300
        self.stopEndSpeed = 0
        self.stopDecelRate = 0
        self.stopRunCurrent = 0
        self.stopHoldCurrent = 0
        self.stopDecelCurrent = 0
        self.stopDelay = 50
        try:
            self.serial = actuator_serial_device
        except IndexError:
            print("No available COM port found! Either not plugged in or "
                  "being used by other program.",
                  file=sys.stderr)
            self.has_pump = False
        except AttributeError:
            print("No available COM port found! Either not plugged in or "
                  "being used by other program.",
                  file=sys.stderr)
            self.has_pump = False
        except SerialException:
            print("No available COM port found! Either not plugged in or "
                  "being used by other program.",
                  file=sys.stderr)
            self.has_pump = False
        else:
            self.has_pump = True

    @property
    def pos(self):
        """
        int: Position of the motor between -18446744073709551616 and
        18446744073709551615
        """
        return int(self._read_current_position())

    def __del__(self):
        if self.has_pump:
            self._stop()
            self.serial.close()

    @staticmethod
    def _initialize_serial(port):
        """
        Initializes the serial port and return the serial.Serial object.

        The serial object is configured with ``baudrate=9600``, ``parity=
        serial.PARITY_NONE``, ``stopbits=serial.STOPBITS_ONE`` and ``bytesize=
        serial.EIGHTBITS``.

        Parameters
        ----------
        port : str
            A string defining the serial port, e.g.: ``'COM3'``

        Returns
        -------
        serial.Serial
            Returns the serial interface object.
        """
        ser = serial.Serial(
            port=port, baudrate=57600, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        ser.isOpen()
        return ser

    def init_pump(self):
        """
        Initializes the linear actuator.
        Enables fluidics_control.py to check if the actuator is connected and powered.
        """
        self.read_firmware_version()

    def _send_to_idea(self, message):
        """
        Sends a message to the motor.
        There is an initial waiting to make sure the pump received the message.

        Parameters
        ----------
        message : str
            The message formats are specified in
            http://haydonkerk.com/Portals/0/IDEA%20Support/IDEA%202013%20Manuals/
            IDEA%20Drive%20Communication%20Manual.pdf`_.
        """
        formatted_message = message + '\r'
        self.serial.write(formatted_message)
        time.sleep(.3)
        return(formatted_message)

    def _wait_for_movement_to_end(self):
        """
        Continues asking the motor is moving until the motor stops moving.

        Notes
        -----
        The motor responds "'oYES[cr]'o#[cr]" while it is moving
        and responds "'oNO[cr]'o#[cr]" when not moving.
        This "No" response sets the movement_checker variable to false, which
        causes the program to clear the serial buffer and exit this function.
        If there is no data in the serial communication buffer, the motor is probably not plugged in,
        so an error message is generated.
        This function is based on the C++ code found here...
        https://github.com/vrpn/vrpn/blob/9fa0ab3676a43527301c9efd3637f80220eb9462/vrpn_IDEA.C
        """
        movement_checker = True
        while movement_checker:
            self._send_to_idea('o')
            if self.serial.inWaiting() == 0:
                raise CheckMotorPluggedIn("Motor not responding, might not be powered")
            while self.serial.inWaiting() and movement_checker:
                ret = str(self.serial.read(1))
                if 'N' in ret:
                    movement_checker = False
        self._clear_buffer()
        return True

    def shutdown(self):
        if self.has_pump:
            self._stop()
            self.serial.close()

    def plunger_move_by(self, incr):
        """
        Moves the plunger ``incr`` steps from the current position.

        Parameters
        ----------
        incr : int
            The number of steps to move the actuator, can be negative.
        """
        message = 'I{}, {}, {}, {}, {}, {}, {}, {}, ' \
                  '{}, {}, {}, {}'.format(
                                incr,
                                self.plunger_speed,
                                self.plunger_speed,
                                self.plunger_speed,
                                self.accelRate,
                                self.decelRate,
                                self.runCurrent,
                                self.holdCurrent,
                                self.accelCurrent,
                                self.decelCurrent,
                                self.delay,
                                self.incrementsPerStep
                                )
        self._send_to_idea(message)
        self._wait_for_movement_to_end()
        self._stop()
        return(message)

    def plunger_move_to(self, position):
        """
        Moves the plunger to an absolute position.

        Parameters
        ----------
        position : int
            The absolute position to move the pump to.

        """
        message = 'M{}, {}, {}, {}, {}, {}, {}, {}, '\
                            '{}, {}, {}, {}'.format(
                                position,
                                self.plunger_speed,
                                self.plunger_speed,
                                self.plunger_speed,
                                self.accelRate,
                                self.decelRate,
                                self.runCurrent,
                                self.holdCurrent,
                                self.accelCurrent,
                                self.decelCurrent,
                                self.delay,
                                self.incrementsPerStep
                                )
        self._send_to_idea(message)
        self._wait_for_movement_to_end()
        self._stop()
        return message

    def set_plunger_speed(self, speed):
        """
        Set the actuator speed.

        Parameters
        ----------
        speed : int
            The actuator speed.
        """
        self.plunger_speed = speed

    def _clear_buffer(self):
        """
        Clears the serial buffer.
        """
        while self.serial.inWaiting():  # is there anything to read?
            self.serial.read()  # if yes, read it and don't do anything.
        return True

    def _stop(self):
        message = 'H{},{},{},{},{},{},{}'.format(
                            self.stopEndSpeed,
                            self.stopDecelRate,
                            self.stopRunCurrent,
                            self.stopHoldCurrent,
                            self.stopDecelCurrent,
                            self.stopDelay,
                            self.incrementsPerStep
                            )
        self._send_to_idea(message)
        return message

    def _go_at_speed(self, speed):
        message = 'Q{},1200,2000,40000,100000,490,490,490,490,50,'\
                            '8'.format(speed)
        self._send_to_idea(message)
        return message

    def read_message(self):
        """
        Parses serial output.
        All messages end with a carriage return (\r).
        So, this method looks for a carriage return and
        returns everything after the single-character
        command that was sent to the motor. Also strips the carriage return
        from the end of the message.
        """
        out = ''
        while True:
            out += self.serial.read(1)
            if '\r' in out and out != '`':
                message = out[2:-1]
                self._clear_buffer()
                return message

    def _abort(self):
        message = 'A'
        self._send_to_idea(message)
        return message

    def read_IO(self):
        self._send_to_idea(':')
        return self.read_message()

    def read_firmware_version(self):
        self._send_to_idea('v')
        return self.read_message()

    def read_drive_number(self):
        self._send_to_idea('k')
        return self.read_message()

    def read_max_current(self):
        self._send_to_idea('j')
        return self.read_message()

    def _read_current_position(self):
        """Queries the controller for the current position."""
        self._send_to_idea('l')
        return self.read_message()

    def set_position_to_zero(self):
        message = 'Z0'
        self._send_to_idea(message)
        return(message)

    def set_outputs(self):
        message = 'O1'
        self._send_to_idea(message)
        return message

# This class is a mock serial object to mimic the Haydon Kerk linear actuator serial device.

# Mock serial object
class MockActuatorSerialDevice(object):
    def __init__(self):
        self.messages = []
        self.read_firmware_version_called = False
        self._stop_called = False
        self._close_called = False
        self.clear_buffer_called = False
        self.mock_an_empty_buffer = False
        self.wait_for_movement_to_end_called = False
        self.wait_output = None

    def close(self):
        self._close_called = True

    def write(self, message):
        """
        Simulates sending a message to the actuator controller.
        Also records whether certain functions were called in the stack
        to confirm that the class method being tested called all of the functions
        that it should call. For example, plunger_move_by() should call _stop().
        """
        self.messages.append(message)
        self.check_if_a_function_was_called()

    def inWaiting(self):
        """
        Mocks an empty buffer so that test functions such as test_read_message()
        and test_wait_for_movement_to_end() don't get stuck when clear_buffer is called and
        reads a buffer that never empties.
        Also mocks an empty buffer to test that the error exception is thrown in wait_for_movement_to_end()
        when the serial buffer is empty
        Simulates a buffer that never empties when testing that the clear buffer method clears a non-empty buffer
        """
        if inspect.stack()[1][3] == '_clear_buffer' or self.mock_an_empty_buffer == True:
            if inspect.stack()[2][3] == 'clear_buffer_gets_stuck_when_buffer_is_not_empty_timeout':
                return True # mock a serial buffer that never empties
            self.check_if_a_function_was_called()
            return False # mock an empty serial buffer
        else:
            return True  # mock a serial buffer that never empties

    def check_if_a_function_was_called(self):
        """
        Also, records whether certain functions were called in the stack
        to indicate that the class method being tested called all of the functions
        that it should call.
        """
        if inspect.stack()[2][3] == '_clear_buffer':
            self._clear_buffer_called = True
        if inspect.stack()[3][3] == "_stop":
            self._stop_called = True
        if inspect.stack()[3][3] == "read_firmware_version":
            self.read_firmware_version_called = True
        if inspect.stack()[2][3] == "_wait_for_movement_to_end":
            self.wait_for_movement_to_end_called = True

    def read(self, int=1):
        """
        Mimics the actuator controller's response for given commands.
        """
        self.check_if_a_function_was_called()
        if inspect.stack()[2][3] == "test_read_message":
            return('__test_message_read\r')
        elif self.messages[-1] == 'o\r':
            return self.wait_output
        elif self.messages[-1] == 'v\r':
            return('__version 1.2.3\r')
        elif self.messages[-1] == ':\r':
            return('__IO_read\r')
        elif self.messages[-1] == 'k\r':
            return('__drive_number_read\r')
        elif self.messages[-1] == 'j\r':
            return ('__max_current_read\r')
        elif self.messages[-1] == 'l\r':
            return('__12345\r')

if __name__ == "__main__":
    pcm4826e_control = Pcm4826e()
    pcm4826e_control.init_pump()
    pcm4826e_control.plunger_move_by(100000)
    pcm4826e_control.plunger_move_to(0)
    del pcm4826e_control