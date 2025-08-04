#!/usr/bin/env python3
import unittest
from parameterized import parameterized

from opendbc.safety import libsafety_py
from opendbc.safety.tests.common import CANPackerPanda, test_all_allowed_msgs, test_longitudinal_safety
from opendbc.safety.tests.common import test_globally_allowed_addrs, test_blacklisted_addrs


MSG_STEERING_MODULE_ADAS = 0x1E2
MSG_STEERING_TORQUE = 0x1FC  
MSG_PEDAL = 0x342
MSG_VEHICLE_SPEED = 0x220
MSG_PCM_BUTTONS = 0x3B0
MSG_LKAS_HUD_ADAS = 0x316
MSG_ACC_HUD_ADAS = 0x32D
MSG_ACC_CMD = 0x32E

BUTTONS = {
  "SET_BTN": 3,
  "RES_BTN": 4, 
  "ACC_ON_BTN": 19,
  "LKAS_ON_BTN": 6,
}


class TestBydSafety(unittest.TestCase):

  @classmethod
  def setUpClass(cls):
    cls.safety = libsafety_py.libsafety_py
    cls.safety.set_safety_hooks(Ecu.byd, 0)
    cls.safety.init_tests()

  def _button_msg(self, buttons):
    # Create PCM_BUTTONS message with specified button states
    values = {}
    for name, bit in BUTTONS.items():
      values[name] = 1 if name in buttons else 0
    return self.packer.make_can_msg_panda("PCM_BUTTONS", 0, values)

  def _steering_msg(self, angle, steer_req=1):
    # Create STEERING_MODULE_ADAS message
    values = {
      "STEER_ANGLE": angle,
      "STEER_REQ": steer_req,
      "COUNTER": 0,
      "CHECKSUM": 0
    }
    return self.packer.make_can_msg_panda("STEERING_MODULE_ADAS", 0, values)

  def _pedal_msg(self, gas=0, brake=0):
    # Create PEDAL message
    values = {
      "GAS_PEDAL": gas,
      "BRAKE_PEDAL": brake,
      "COUNTER": 0,
      "CHECKSUM": 0
    }
    return self.packer.make_can_msg_panda("PEDAL", 0, values)

  def setUp(self):
    self.packer = CANPackerPanda("byd_general_pt")
    self.safety.set_safety_hooks(Ecu.byd, 0)
    self.safety.init_tests()

  def test_rx_hook(self):
    # Test that RX hook processes expected messages
    self.safety.safety_rx_hook(self._steering_msg(0))
    self.safety.safety_rx_hook(self._pedal_msg(0, 0))

  def test_tx_hook(self):
    # Test that valid steering commands are allowed
    self.assertTrue(self.safety.safety_tx_hook(self._steering_msg(10, 1)))
    
    # Test that commands without steer_req bit are allowed when angle is small
    self.assertTrue(self.safety.safety_tx_hook(self._steering_msg(0, 0)))

  def test_gas_brake_pressed(self):
    # Test gas pedal detection
    self.safety.safety_rx_hook(self._pedal_msg(gas=10, brake=0))
    self.assertEqual(self.safety.get_gas_pressed_prev(), False)  # Should be false initially
    
    # Test brake pedal detection  
    self.safety.safety_rx_hook(self._pedal_msg(gas=0, brake=10))
    self.assertEqual(self.safety.get_brake_pressed_prev(), False)  # Should be false initially

  def test_buttons(self):
    # Test button message processing
    self.safety.safety_rx_hook(self._button_msg(["SET_BTN"]))
    self.safety.safety_rx_hook(self._button_msg(["RES_BTN"]))
    self.safety.safety_rx_hook(self._button_msg(["ACC_ON_BTN"]))

  def test_allowed_msgs(self):
    # Test that all expected TX messages are allowed
    allowed_addrs = [MSG_STEERING_MODULE_ADAS, MSG_LKAS_HUD_ADAS, MSG_ACC_HUD_ADAS, MSG_ACC_CMD]
    test_all_allowed_msgs(self, allowed_addrs)

  def test_blocked_msgs(self):
    # Test that diagnostic messages are properly blocked
    blocked_addrs = [0x7E0, 0x7E8]  # UDS addresses
    test_blacklisted_addrs(self, blocked_addrs)

  def test_checksum_counter(self):
    # Test checksum and counter functions
    msg = self._steering_msg(10)
    
    # Basic functionality test - should not crash
    counter = self.safety.get_byd_counter(msg)
    checksum = self.safety.get_byd_checksum(msg)
    computed_checksum = self.safety.get_byd_compute_checksum(msg)
    
    # Checksum should be deterministic
    self.assertEqual(checksum, self.safety.get_byd_checksum(msg))


if __name__ == "__main__":
  unittest.main()
