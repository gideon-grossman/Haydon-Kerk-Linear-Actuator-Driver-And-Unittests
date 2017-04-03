from omnidaq.omnifluidics_project.omnifluidics.IDEA_actuator import Pcm4826e, MockActuatorSerialDevice, CheckMotorPluggedIn
import unittest
from threading import Thread
import functools
import random
import string

def timeout(timeout):
    """
    creates a timeout decorator
    """
    def deco(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            res = [Exception('function [%s] timeout [%s seconds] exceeded!' % (func.__name__, timeout))]
            def newFunc():
                try:
                    res[0] = func(*args, **kwargs)
                except Exception, e:
                    res[0] = e
            t = Thread(target=newFunc)
            t.daemon = True
            try:
                t.start()
                t.join(timeout)
            except Exception, je:
                print ('error starting thread')
                raise je
            ret = res[0]
            if isinstance(ret, BaseException):
                raise ret
            return ret
        return wrapper
    return deco


def test_init_pump():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    actuator_tester.init_pump()
    assert actuator_tester.serial.read_firmware_version_called

def test_send_to_idea():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    test_message = "test_message"
    assert actuator_tester._send_to_idea(test_message) == test_message + '\r'

def test_wait_for_movement_to_end():
    """
    Tests that the _wait_for_movement_to_end method returns when the serial device
    replies with an 'N'
    """
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    actuator_tester.serial.wait_output = 'N'
    assert actuator_tester._wait_for_movement_to_end()

def test_wait_for_movement_to_end_does_not_end_without_an_end_signal():
    """
    Tests that the _wait_for_movement_to_end method does not return when the serial device
    does not reply with an 'N' (i.e. while the actuator is still moving)
    """
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    ascii_characters_without_N = string.ascii_letters.replace('N','')
    actuator_tester.serial.wait_output = str(random.choice(ascii_characters_without_N))
    timed_out = False
    try:
        wait_for_movement_to_end_does_not_end_without_an_end_signal_timeout(actuator_tester)
    except:
        timed_out = True
    assert timed_out == True

@timeout(2)
def wait_for_movement_to_end_does_not_end_without_an_end_signal_timeout(actuator_tester):
    actuator_tester._wait_for_movement_to_end()

class UnittestClass(unittest.TestCase):
    """
    The purpose of this class is to allow the error exception test below to run.
    """
    def test_wait_for_movement_to_end_throws_motor_disconnected_error_exception(self):
        """
        Tests whether the wait_for_movement_to_end() method raises an error if the motor is not plugged in.
        This test times out if the method fails to raise an error.
        """
        actuator_serial_device = MockActuatorSerialDevice()
        actuator_tester = Pcm4826e(actuator_serial_device)
        actuator_tester.serial.mock_an_empty_buffer = True
        with self.assertRaises(CheckMotorPluggedIn):
            actuator_tester._wait_for_movement_to_end()

def test_shutdown():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    actuator_tester.shutdown()
    assert actuator_tester.serial._stop_called
    assert actuator_tester.serial._close_called



def test_move_by():
    # Tests whether the plunger_move_by method sends the correct instructions to the actuator
    # Also tests that the plunger_move_by method calls _wait_for_movement_to_end() and _stop()
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    increments = 10000
    assert actuator_tester.plunger_move_by(increments) == 'I{}, 36571, 36571,' \
                                                 ' 36571, 16777215, 16777215,' \
                                                 ' 245, 245, 245, 245, 300,' \
                                                 ' 64'.format(increments)
    assert actuator_tester.serial.wait_for_movement_to_end_called
    assert actuator_tester.serial._stop_called

def test_move_to():
    # Tests whether the plunger_move_to function sends the correct instructions to the actuator
    # Also tests that the plunger_move_to method calls _wait_for_movement_to_end() and _stop()
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    position = 12345
    assert actuator_tester.plunger_move_to(position) == 'M{}, 36571, 36571, 36571, 16777215,' \
                                                 ' 16777215, 245, 245, 245, 245,' \
                                                 ' 300, 64'.format(position)
    assert actuator_tester.serial.wait_for_movement_to_end_called
    assert actuator_tester.serial._stop_called

def test_set_plunger_speed():
    new_speed = 54321
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    actuator_tester.set_plunger_speed(new_speed)
    assert actuator_tester.plunger_speed == new_speed

def test_clear_buffer_returns_true_when_buffer_is_empty():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    assert actuator_tester._clear_buffer()

def test_clear_buffer_gets_stuck_when_buffer_is_not_empty():
    """
    Tests that the clear_buffer method does not return when the buffer is not empty
    """
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    timed_out = False
    try:
        clear_buffer_gets_stuck_when_buffer_is_not_empty_timeout(actuator_tester)
    except:
        timed_out = True
    assert timed_out == True

@timeout(2)
def clear_buffer_gets_stuck_when_buffer_is_not_empty_timeout(actuator_tester):
    actuator_tester._clear_buffer()


def test_stop():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    actuator_tester._stop()
    assert actuator_tester.serial.messages[0] == 'H{},{},{},{},{},{},{}\r'.format(
                            actuator_tester.stopEndSpeed,
                            actuator_tester.stopDecelRate,
                            actuator_tester.stopRunCurrent,
                            actuator_tester.stopHoldCurrent,
                            actuator_tester.stopDecelCurrent,
                            actuator_tester.stopDelay,
                            actuator_tester.incrementsPerStep)
def test_go_at_speed():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    speed = 23145
    assert actuator_tester._go_at_speed(speed) == 'Q{},1200,2000,40000,100000,490,490,490,490,50,8'.format(speed)

def test_read_message():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    actuator_tester.serial.messages.append("test_message")
    assert actuator_tester.read_message() == "test_message_read"
    assert actuator_tester.serial._clear_buffer_called == True

def test_abort():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    assert actuator_tester._abort() == 'A'
    assert actuator_tester.serial.messages[0] == 'A\r'

def test_readIO():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    assert actuator_tester.read_IO() == "IO_read"

def test_read_firmware_version():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    assert actuator_tester.read_firmware_version() == 'version 1.2.3'

def test_read_drive_number():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    assert actuator_tester.read_drive_number() == 'drive_number_read'

def test_read_max_current():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    assert actuator_tester.read_max_current() == 'max_current_read'

def test_read_current_position():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    assert actuator_tester._read_current_position() == '12345'

def test_set_position_to_zero():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    assert actuator_tester.set_position_to_zero() == 'Z0'

def test_set_outputs():
    actuator_serial_device = MockActuatorSerialDevice()
    actuator_tester = Pcm4826e(actuator_serial_device)
    assert actuator_tester.set_outputs() == 'O1'