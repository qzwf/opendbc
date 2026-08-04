#!/usr/bin/env python3
import unittest

from opendbc.car.byd.bydcan import byd_checksum, CHECKSUM_KEY
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

STEERING_MODULE_ADAS = 0x1E2
LKAS_HUD_ADAS = 0x316
ACC_CMD = 0x32E  # blocked: not in the TX allowlist
ACC_HUD_ADAS = 0x32D

MAIN_BUS = 0
CAM_BUS = 2


def sign(msg):
  """Re-pack a message with a valid BYD checksum in the last byte."""
  dat = bytes(bytearray(msg.data[0:8]))
  dat = dat[:-1] + bytes([byd_checksum(CHECKSUM_KEY, dat[:-1])])
  return libsafety_py.make_CANPacket(msg.addr, msg.bus, dat)


class TestBydSafetyBase(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  TX_MSGS = [[STEERING_MODULE_ADAS, MAIN_BUS], [LKAS_HUD_ADAS, MAIN_BUS]]
  RELAY_MALFUNCTION_ADDRS = {MAIN_BUS: (STEERING_MODULE_ADAS, LKAS_HUD_ADAS)}
  FWD_BLACKLISTED_ADDRS = {CAM_BUS: [STEERING_MODULE_ADAS, LKAS_HUD_ADAS]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  STEER_ANGLE_MAX = 90
  # sweep stays inside the EPS limit: enforce_angle_error clamps commands to max_angle
  STEER_ANGLE_TEST_MAX = 80
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 25.]
  ANGLE_RATE_UP = [2.5, 1.5, .4]
  ANGLE_RATE_DOWN = [2.5, 1.5, .6]

  def setUp(self):
    self.packer = CANPackerSafety("byd_general")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"STEER_ANGLE": angle, "STEER_REQ": enabled, "STEER_REQ_ACTIVE_LOW": not enabled,
              "SET_ME_X01": 1 if enabled else 0, "SET_ME_XE": 0xB if enabled else 0}
    return self.packer.make_can_msg_safety("STEERING_MODULE_ADAS", MAIN_BUS, values)

  def _angle_meas_msg(self, angle: float):
    values = {"STEER_ANGLE_2": angle}
    return self.packer.make_can_msg_safety("STEER_MODULE_2", MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ON1": enable, "ACC_ON2": enable}
    return sign(self.packer.make_can_msg_safety("ACC_HUD_ADAS", CAM_BUS, values))

  def _speed_msg(self, speed):
    # carstate applies a 40/53 correction to the raw DBC value; mirror it here
    kph = speed * 3.6 * (53.0 / 40.0)
    values = {"WHEELSPEED_FL": kph, "WHEELSPEED_FR": kph, "WHEELSPEED_BL": kph}
    return self.packer.make_can_msg_safety("WHEEL_SPEED", MAIN_BUS, values)

  def _speed_msg_2(self, speed: float):
    return None

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL": 0.5 if brake else 0.0}
    return sign(self.packer.make_can_msg_safety("PEDAL", MAIN_BUS, values))

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": gas}
    return sign(self.packer.make_can_msg_safety("PEDAL", MAIN_BUS, values))

  def test_steer_req_bit_mismatch(self):
    """STEER_REQ and its active-low twin must always disagree."""
    self.safety.set_controls_allowed(True)
    for req, active_low in ((1, 1), (0, 0)):
      values = {"STEER_ANGLE": 0, "STEER_REQ": req, "STEER_REQ_ACTIVE_LOW": active_low}
      msg = self.packer.make_can_msg_safety("STEERING_MODULE_ADAS", MAIN_BUS, values)
      self.assertFalse(self._tx(msg))

  def test_checksum_rejects_corrupt_frames(self):
    """A frame whose checksum doesn't match must invalidate rx and drop controls."""
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._rx(self._user_brake_msg(False)))

    bad = self.packer.make_can_msg_safety("PEDAL", MAIN_BUS, {"BRAKE_PEDAL": 0.0})
    bad.data[7] = 0xAA  # deliberately wrong checksum
    self.assertFalse(self._rx(bad))
    self.assertFalse(self.safety.get_controls_allowed())


class TestBydSafety(TestBydSafetyBase):
  def test_acc_cmd_not_allowed(self):
    """Longitudinal is stock-only; ACC_CMD must never be transmitted."""
    self.safety.set_controls_allowed(True)
    values = {"ACCEL_CMD": 100}
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("ACC_CMD", MAIN_BUS, values)))


if __name__ == "__main__":
  unittest.main()
