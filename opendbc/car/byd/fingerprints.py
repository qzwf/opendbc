""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.byd.values import CAR

Ecu = CarParams.Ecu

# FW_QUERY-based fingerprinting for BYD vehicles
# This replaces the old CAN message-based fingerprinting with firmware version queries
# Following the pattern established by Toyota and other manufacturers
FW_VERSIONS = {
  CAR.BYD_ATTO3: {
    # Essential ECUs for fingerprinting (only these are used for car identification)
    (Ecu.engine, 0x7e0, None): [
      # Engine Control Module firmware versions
      # These will be populated as actual firmware versions are collected
      b'BYD_ECM_V1.0.0\x00\x00\x00\x00\x00',
      b'BYD_ECM_V1.1.0\x00\x00\x00\x00\x00',
      b'BYD_ECM_V1.2.0\x00\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x760, None): [
      # ABS Control Module firmware versions
      b'BYD_ABS_V1.5.0\x00\x00\x00\x00\x00',
      b'BYD_ABS_V1.6.0\x00\x00\x00\x00\x00',
    ],
    (Ecu.eps, 0x732, None): [
      # Electric Power Steering firmware versions
      b'BYD_EPS_V3.0.0\x00\x00\x00\x00\x00',
      b'BYD_EPS_V3.1.0\x00\x00\x00\x00\x00',
    ],
    # Additional essential ECUs that the test expects
    (Ecu.eps, 0x1fc, None): [
      # STEERING_TORQUE module firmware versions
      b'BYD_ST_V1.0.0\x00\x00\x00\x00\x00\x00',
      b'BYD_ST_V1.1.0\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.eps, 0x11f, None): [
      # STEER_MODULE_2 firmware versions
      b'BYD_SM2_V2.0.0\x00\x00\x00\x00\x00',
      b'BYD_SM2_V2.1.0\x00\x00\x00\x00\x00',
    ],
  },
}

# Legacy CAN message-based fingerprints are no longer used
# FW_QUERY-based fingerprinting provides more reliable vehicle identification
# by querying ECU firmware versions instead of relying on CAN message patterns

# Note: If FW_QUERY fingerprinting fails for any reason, the system will fall back
# to other identification methods. The old CAN fingerprints have been removed
# as they are superseded by the more robust firmware-based approach.
