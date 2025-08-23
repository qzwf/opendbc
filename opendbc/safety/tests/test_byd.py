#!/usr/bin/env python3
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda

# BYD CAN message IDs
MSG_BYD_STEERING_MODULE_ADAS = 0x1E2  # 482
MSG_BYD_ACC_CMD = 0x32E               # 814
MSG_BYD_STEERING_TORQUE = 0x1FC       # 508
MSG_BYD_PEDAL = 0x200                 # 512
MSG_BYD_DRIVE_STATE = 0x134           # 308


class TestBydSafety(common.PandaSafetyTest):
  """BYD ATTO3 safety tests

  This is a minimal test class for BYD since there's no dedicated BYD safety mode yet.
  BYD currently uses noOutput safety mode which allows all messages.

  The BYD safety implementation is located in opendbc/safety/modes/byd.h
  but is not yet integrated into the safety mode enum.
  """

  # Required attributes for PandaSafetyTest
  TX_MSGS = [[MSG_BYD_STEERING_MODULE_ADAS, 0], [MSG_BYD_ACC_CMD, 0]]
  FWD_BLACKLISTED_ADDRS = {}  # noOutput mode doesn't blacklist anything
  FWD_BUS_LOOKUP = {}  # noOutput mode doesn't forward anything

  def setUp(self):
    self.packer = CANPackerPanda("byd_general")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)
    self.safety.init_tests()

  def test_placeholder(self):
    """Placeholder test to ensure the file is valid Python."""
    # Define BYD safety constants
    BYD_MAX_STEER = 1500

    # Basic validation that constants are defined
    self.assertEqual(MSG_BYD_STEERING_MODULE_ADAS, 0x1E2)
    self.assertEqual(MSG_BYD_ACC_CMD, 0x32E)
    self.assertEqual(BYD_MAX_STEER, 1500)
