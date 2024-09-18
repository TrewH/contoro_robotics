# external modules
import unittest
from unittest.mock import patch, MagicMock

# internal modules
from unloading_robot_hal.roboteq_driver import Roboteq  # type: ignore


# Class to test roboteq methods.
class test_roboteq_driver(unittest.TestCase):

    # Mock the serial.Serial class.
    @patch("serial.Serial")
    def setUp(self, mock_serial: MagicMock):

        # Mock serial instance.
        self.mock_serial_instance = MagicMock()
        mock_serial.return_value = self.mock_serial_instance
        self.mock_serial_instance.in_waiting = 0

        # Simulate open port.
        self.mock_serial_instance.is_open = True

        # Creates instance of roboteq controllerMagicMock().
        self.controller = Roboteq("COM3")

    def test_setSpeed(self):
        """Compares a sent speed to the recieved serial message"""
        # Set speed; serial command sent is b'!G 1 10\r'.
        self.controller.setSpeed(10, 1)
        # Check if serial port recieved proper string.
        self.mock_serial_instance.write.assert_called_with(b"!G 1 10\r")

    def test_close(self):
        """Checks if close() was sent and motor speed is 0"""
        # Checks to see if close() is called
        self.controller.close()
        self.mock_serial_instance.close.assert_called()
        # Check motor speeds set to 0
        self.mock_serial_instance.write.assert_any_call(b"!G 1 0\r")
        self.mock_serial_instance.write.assert_any_call(b"!G 2 0\r")

    def test_send(self):
        """Compares a sent message to what the port recieves"""
        # Sends 'test' message to serial port.
        self.controller._send("test")
        # Check if serial port recieved the right message.
        self.mock_serial_instance.write.assert_called_with(b"test\r")

    def test_sendSuccess(self):
        """Confirms if sending a command gets recieved by controller"""
        # Imitate controller response when receiving a command that
        # should not return anything specific.
        self.mock_serial_instance.read.return_value = b"+\r"
        # Test sendSuccess()
        response = self.controller._sendSuccess()
        # Check sendSuccess() is returning True
        self.assertTrue(response)

    def test_readResponse(self):
        """Test if response is read correctly"""
        # Set return value .
        self.mock_serial_instance.read.return_value = b"Hello\r"
        # Set response equal to what the roboteq reads.
        response = self.controller._readResponse()
        # Check if response is equal to expected.
        self.assertEqual(response, "Hello\r")

    def test_stopMotor(self):
        """Confirms stop motor stops a single motor properly"""
        # Sets response behavior if send works.
        self.mock_serial_instance.read.return_value = b"+\r"
        # Confirm stopMotor() returned True.
        self.assertTrue(self.controller.stopMotor(1))
        # Confirm stopMotor() wrote proper message.
        self.mock_serial_instance.write.assert_called_with(b"!MS 1\r")
        # In case of failure, testing manually set speed to 0
        self.mock_serial_instance.read.side_effect = [
            b"failcase\r",
            b"+\r",
        ]
        self.assertTrue(self.controller.stopMotor(1))
        self.mock_serial_instance.write.assert_called_with(b"!G 1 0\r")

    def test_stop(self):
        """Confirms stop properly stops both motors"""
        # Sets response behavior if send works.
        self.mock_serial_instance.read.return_value = b"+\r"
        # Stops both motors 1 and 2.
        response = self.controller.stop()
        # Confirm stop() returned True for stopMotor() for 1 and 2.
        self.assertTrue(response)
        # Confirm commands to stop both motors were sent
        self.mock_serial_instance.write.assert_any_call(b"!MS 1\r")
        self.mock_serial_instance.write.assert_any_call(b"!MS 2\r")

    def test_estop(self):
        """Confirms estop functions properly"""
        # Sets response behavior if send works.
        self.mock_serial_instance.read.return_value = b"+\r"
        # Confirm estop() returned true.
        self.assertTrue(self.controller.estop())
        # Confirm proper command sent to driver.
        self.mock_serial_instance.write.assert_any_call(b"!EX\r")

    def test_releaseEstop(self):
        """Confirms releaseestop functions properly"""
        # Sets response behavior if send works.
        self.mock_serial_instance.read.return_value = b"+\r"
        # Confirm estop() returned true.
        self.assertTrue(self.controller.releaseEstop())
        # Confirm proper command sent to driver.
        self.mock_serial_instance.write.assert_any_call(b"!MG\r")

    def test_readFaultFlags(self):
        """Confirms fault flags are read correctly"""
        with patch("rospy.logerr") as mock_logerr:
            # 1 = Overheat fault.
            self.mock_serial_instance.read.return_value = b"FF=1\r"
            self.assertTrue(self.controller.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overheat fault.')",
            )
            # 2 = Overvoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=2\r"
            self.assertTrue(self.controller.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overvoltage fault.')",
            )
            # 3 = Overheat overvoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=3\r"
            self.assertTrue(self.controller.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overheat Overvoltage fault.')",
            )
            # 4 = Undervoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=4\r"
            self.assertTrue(self.controller.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Undervoltage fault.')",
            )
            # 5 = Overheat Undervoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=5\r"
            self.assertTrue(self.controller.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overheat Undervoltage fault.')",
            )
            # 6 = Overvoltage Undervoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=6\r"
            self.assertTrue(self.controller.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overvoltage Undervoltage fault.')",
            )
            # 7 = Overheat Overvoltage Undervoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=7\r"
            self.assertTrue(self.controller.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overheat Overvoltage Undervoltage fault.')",
            )
            # 8 = Short circuit fault.
            self.mock_serial_instance.read.return_value = b"FF=8\r"
            self.assertTrue(self.controller.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Short circuit fault.')",
            )

    def test_readMotorSpeedRPM(self):
        """Confirm motor RPM can be properly read"""
        # Pretend roboteq respons with 'BS=100'.
        self.mock_serial_instance.read.return_value = b"BS=100\r"
        # Set response equal to what RPM is read.
        response = self.controller.readMotorSpeedRPM(1)
        # Confirm read response matches the 100 RPM inputed.
        self.assertEqual(response, 100)
        # Confirm proper command sent to driver.
        self.mock_serial_instance.write.assert_any_call(b"?BS 1\r")

    def test_readMotorSpeedRatioRPM(self):
        """Confirm motor ratio RPM can be properly read"""
        # Pretend roboteq respons with 'BS=100'.
        self.mock_serial_instance.read.return_value = b"BSR=100\r"
        # Set response equal to what RPM is read.
        response = self.controller.readMotorSpeedRatioRPM(1)
        # Confirm read response matches the 100 RPM inputed.
        self.assertEqual(response, 100)
        # Confirm proper command sent to driver.
        self.mock_serial_instance.write.assert_any_call(b"?BSR 1\r")

    def test_readWheelSpeedRPM(self):
        """Confirm wheel speed is calculated from gear ratio properly"""
        # Pretend roboteq responds with 'BS=100'.
        self.mock_serial_instance.read.return_value = b"BS=160\r"
        # Assuming gear ratio of 160, get calculated wheel speed.
        response = self.controller.readWheelSpeedRPM()
        # Confirm calculated speed is correct.
        self.assertEqual(response, 1)
        # Confirm proper command sent to driver.
        self.mock_serial_instance.write.assert_any_call(b"?BS 1\r")

    def test_reset(self):
        """Tests whether serial port is closed and reopened"""
        # Get state prior to reset (open, True).
        state1 = self.mock_serial_instance.is_open
        # Call controller reset.
        self.controller.reset()
        # Get state after reset.
        state2 = self.mock_serial_instance.is_open
        # Check both states were true/open.
        self.assertEqual(state1, state2)
        # Check that close() was called.
        self.mock_serial_instance.close.assert_called()

    def test_readbatteryVoltage(self):
        """Tests battery voltage is read and calculated properly"""
        # Pretend roboteq responds with 'V=100'
        self.mock_serial_instance.read.return_value = b"V=100\r"
        # Get readBatteryVoltage response
        response = self.controller.readBatteryVoltage()
        # Check response equals expected value (1/10th of input)
        self.assertEqual(response, 10)
        # Confirm proper command sent to driver.
        self.mock_serial_instance.write.assert_any_call(b"?V 2\r")


if __name__ == "__main__":
    unittest.main()
