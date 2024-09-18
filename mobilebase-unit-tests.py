# External modules.
import unittest
from unittest.mock import patch, MagicMock, call

# Internal modules
from unloading_robot_hal.mobile_base import MobileBase  # type: ignore


class test_mobile_base(unittest.TestCase):

    @patch("serial.Serial")
    def setUp(self, mock_serial: MagicMock):
        # Generate mock serial port.
        self.mock_serial_instance = MagicMock()
        mock_serial.return_value = self.mock_serial_instance
        self.mock_serial_instance.in_waiting = 0
        self.mock_serial_instance.is_open = True

        # Creates an instance of mobile base with one driver.
        self.mobilebaseone = MobileBase("COM3")
        # Creates an instance of mobile base with two drivers.
        self.mobilebasetwo = MobileBase("COM3", "COM4")

    def test_setSpeed(self):
        """Tests setSpeed() for a mobile base with one or two drivers"""
        # ONE DRIVER
        self.mock_serial_instance.read.return_value = b"+\r"
        self.mobilebaseone.setSpeed(100, -100)
        self.mock_serial_instance.write.assert_any_call(b"!G 2 -100\r")
        self.mock_serial_instance.write.assert_any_call(b"!G 1 100\r")

        # TWO DRIVERS
        self.mock_serial_instance.read.return_value = b"+\r"
        self.mobilebasetwo.setSpeed(50, -50)
        self.mock_serial_instance.write.assert_any_call(b"!G 1 50\r")
        self.mock_serial_instance.write.assert_any_call(b"!G 2 50\r")
        self.mock_serial_instance.write.assert_any_call(b"!G 1 -50\r")
        self.mock_serial_instance.write.assert_any_call(b"!G 2 -50\r")

    def test_stop(self):
        """Tests stop() for a mobile base with one or two drivers"""
        # ONE DRIVER
        self.mock_serial_instance.read.return_value = b"+\r"
        responseOne = self.mobilebaseone.stop()
        self.assertTrue(responseOne)
        self.mock_serial_instance.write.assert_any_call(b"!MS 1\r")
        self.mock_serial_instance.write.assert_any_call(b"!MS 2\r")

        # TWO DRIVERS
        self.mock_serial_instance.read.return_value = b"+\r"
        responseTwo = self.mobilebasetwo.stop()
        self.assertTrue(responseTwo)
        write_calls = [
            call for call in self.mock_serial_instance.mock_calls if call[0] == "write"
        ]
        expected_write_calls = [
            call.write(b"!MS 1\r"),
            call.write(b"!MS 2\r"),
            call.write(b"!MS 1\r"),
            call.write(b"!MS 2\r"),
            call.write(b"!MS 1\r"),
            call.write(b"!MS 2\r"),
        ]
        self.assertEqual(write_calls, expected_write_calls)
        assert self.mock_serial_instance.write.call_count == 6

    def test_eStop(self):
        """Tests eStop() for a mobile base with one or two drivers"""
        # ONE DRIVER
        self.mock_serial_instance.read.return_value = b"+\r"
        responseone = self.mobilebaseone.eStop()
        self.assertTrue(responseone)
        self.mock_serial_instance.write.assert_called_with(b"!EX\r")

        # TWO DRIVERS
        self.mock_serial_instance.read.return_value = b"+\r"
        responsetwo = self.mobilebasetwo.eStop()
        self.assertTrue(responsetwo)
        self.mock_serial_instance.write.assert_called_with(b"!EX\r")

    def test_releaseEstop(self):
        """Tests releaseEstop() for a mobile base with one or two drivers"""
        # ONE DRIVER
        self.mock_serial_instance.read.return_value = b"+\r"
        responseone = self.mobilebaseone.releaseEstop()
        self.assertTrue(responseone)
        self.mock_serial_instance.write.assert_any_call(b"!MG\r")

        # TWO DRIVERS
        self.mock_serial_instance.read.return_value = b"+\r"
        responsetwo = self.mobilebasetwo.releaseEstop()
        self.assertTrue(responsetwo)
        self.mock_serial_instance.write.assert_any_call(b"!MG\r")

    def test_readFaultFlags(self):
        """Tests readFaultFlags() for a mobile base with one or two drivers"""
        # ONE DRIVER
        with patch("rospy.logerr") as mock_logerr:
            # 1 = Overheat fault.
            self.mock_serial_instance.read.return_value = b"FF=1\r"
            self.assertTrue(self.mobilebaseone.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overheat fault.')",
            )
            # 2 = Overvoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=2\r"
            self.assertTrue(self.mobilebaseone.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overvoltage fault.')",
            )
            # 3 = Overheat overvoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=3\r"
            self.assertTrue(self.mobilebaseone.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overheat Overvoltage fault.')",
            )
            # 4 = Undervoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=4\r"
            self.assertTrue(self.mobilebaseone.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Undervoltage fault.')",
            )
            # 5 = Overheat Undervoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=5\r"
            self.assertTrue(self.mobilebaseone.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overheat Undervoltage fault.')",
            )
            # 6 = Overvoltage Undervoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=6\r"
            self.assertTrue(self.mobilebaseone.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overvoltage Undervoltage fault.')",
            )
            # 7 = Overheat Overvoltage Undervoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=7\r"
            self.assertTrue(self.mobilebaseone.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overheat Overvoltage Undervoltage fault.')",
            )
            # 8 = Short circuit fault.
            self.mock_serial_instance.read.return_value = b"FF=8\r"
            self.assertTrue(self.mobilebaseone.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Short circuit fault.')",
            )

        # TWO DRIVERS
        with patch("rospy.logerr") as mock_logerr:
            # 1 = Overheat fault.
            self.mock_serial_instance.read.return_value = b"FF=1\r"
            self.assertTrue(self.mobilebasetwo.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overheat fault.')",
            )
            # 2 = Overvoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=2\r"
            self.assertTrue(self.mobilebasetwo.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overvoltage fault.')",
            )
            # 3 = Overheat overvoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=3\r"
            self.assertTrue(self.mobilebasetwo.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overheat Overvoltage fault.')",
            )
            # 4 = Undervoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=4\r"
            self.assertTrue(self.mobilebasetwo.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Undervoltage fault.')",
            )
            # 5 = Overheat Undervoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=5\r"
            self.assertTrue(self.mobilebasetwo.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overheat Undervoltage fault.')",
            )
            # 6 = Overvoltage Undervoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=6\r"
            self.assertTrue(self.mobilebasetwo.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overvoltage Undervoltage fault.')",
            )
            # 7 = Overheat Overvoltage Undervoltage fault.
            self.mock_serial_instance.read.return_value = b"FF=7\r"
            self.assertTrue(self.mobilebasetwo.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Overheat Overvoltage Undervoltage fault.')",
            )
            # 8 = Short circuit fault.
            self.mock_serial_instance.read.return_value = b"FF=8\r"
            self.assertTrue(self.mobilebasetwo.readFaultFlags())
            self.mock_serial_instance.write.assert_any_call(b"?FF\r")
            self.assertEqual(
                f"{mock_logerr.call_args_list[-1]}",
                "call('[ERROR] ROBOTEQ: Short circuit fault.')",
            )

    def test_move(self):
        """Tests the move functionality for a mobile base with one or two drivers"""
        # ONE DRIVER
        self.mock_serial_instance.read.side_effect = [
            b"FF=0\r",
            b"FF=0\r",
            b"+\r",
            b"+\r",
        ]
        self.assertTrue(self.mobilebaseone.move(100, 0))
        self.mock_serial_instance.write.assert_any_call(b"!G 2 1000\r")
        self.mock_serial_instance.write.assert_any_call(b"!G 1 -1000\r")

        # TWO DRIVERS
        self.mock_serial_instance.read.side_effect = [
            b"FF=0\r",
            b"FF=0\r",
            b"FF=0\r",
            b"FF=0\r",
            b"+\r",
            b"+\r",
            b"+\r",
            b"+\r",
        ]
        self.assertTrue(self.mobilebasetwo.move(100, 0))
        self.mock_serial_instance.write.assert_any_call(b"!G 1 -1000\r")
        self.mock_serial_instance.write.assert_any_call(b"!G 2 -1000\r")
        self.mock_serial_instance.write.assert_any_call(b"!G 1 1000\r")
        self.mock_serial_instance.write.assert_any_call(b"!G 2 1000\r")

    def test_readMotorSpeedRatio(self):
        """Tests that readMotorSpeedRatio is correct for one and two drivers"""
        # ONE DRIVER
        self.mock_serial_instance.read.side_effect = [
            b"BSR=4\r",
            b"BSR=10\r",
        ]
        [left, right] = self.mobilebaseone.readMotorSpeedRatio()
        self.assertEqual(left, 4)
        self.assertEqual(right, 10)

        # TWO DRIVERS
        self.mock_serial_instance.read.side_effect = [
            b"BSR=6\r",
            b"BSR=8\r",
        ]
        [left, right] = self.mobilebaseone.readMotorSpeedRatio()
        self.assertEqual(left, 6)
        self.assertEqual(right, 8)

    def test_readbatteryVoltage(self):
        """Tests that battery voltage is read and calculated properly.
        readBatteryVoltage() returns 1/10th of value read from driver"""
        # ONE DRIVER
        self.mock_serial_instance.read.return_value = b"V=120\r"
        voltage = self.mobilebaseone.readBatteryVoltage()
        self.assertEqual(voltage, 12)
        self.mock_serial_instance.write.assert_any_call(b"?V 2\r")

        # TWO DRIVERS
        self.mock_serial_instance.read.return_value = b"V=140\r"
        voltage = self.mobilebasetwo.readBatteryVoltage()
        self.assertEqual(voltage, 14)
        self.mock_serial_instance.write.assert_any_call(b"?V 2\r")

    def test_close(self):
        """Tests that close is properly called to driver"""
        # ONE DRIVER
        self.mobilebaseone.close()
        self.mock_serial_instance.close.assert_called()
        self.mock_serial_instance.write.assert_any_call(b"!G 1 0\r")
        self.mock_serial_instance.write.assert_any_call(b"!G 2 0\r")

        # TWO DRIVERS
        self.mobilebasetwo.close()
        self.mock_serial_instance.close.assert_called()
        self.mock_serial_instance.write.assert_any_call(b"!G 1 0\r")
        self.mock_serial_instance.write.assert_any_call(b"!G 2 0\r")


if __name__ == "__main__":
    unittest.main()
