#QZWF

#Updating the code using iXcess Bukapilot as reference

from dataclasses import dataclass, field
from enum import Enum, IntFlag

from panda import uds

from opendbc.car import CarSpecs, PlatformConfig, Platforms, AngleRateLimit, dbc_dict, DbcDict
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarFootnote, CarDocs, Column, CarParts, CarHarness
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries,p16


HUD_MULTIPLIER = 0.718

Ecu = CarParams.Ecu
MIN_ACC_SPEED = 19. * CV.MPH_TO_MS
PEDAL_TRANSITION = 10. * CV.MPH_TO_MS


class CarControllerParams:
  STEER_STEP = 1
  STEER_MAX = 1500
  STEER_ERROR_MAX = 350     # max delta between torque cmd and torque motor

  def __init__(self, CP):
      self.ACCEL_MAX = 2.0
      self.STEER_DELTA_UP = 15       # 1.0s time to peak torque
      self.STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)



@dataclass
class BYDCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))

@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: dict = field(default_factory=lambda: dbc_dict('byd_general_pt', None))

  def init(self):
    self.dbc_dict = dbc_dict('byd_general_pt', None)



class CAR(Platforms):
  #BYD ATTO 3
  BYD_ATTO3 = BYDPlatformConfig(
    [
      BYDCarDocs("BYD ATTO 3")
    ],
  CarSpecs(mass=1750, wheelbase=2.72, steerRatio=14.8, tireStiffnessFactor=0.7983),
  dbc_dict('byd_general_pt', None),
  )

"""
FINGERPRINTS = {
  CAR.BYD_ATTO3: [{
    85: 8, 140: 8, 204: 20, 213: 8, 287: 5, 289: 8, 290: 8, 291: 8, 301: 8, 307: 8, 309: 8, 324: 8, 327: 8, 337: 8, 356: 8, 371: 8, 384: 8, 385: 3, 410: 8, 418: 8, 450: 8, 482: 8, 496: 8, 508: 8, 522: 8, 536: 8, 537: 8, 544: 8, 546: 8, 547: 8, 576: 8, 577: 8, 578: 8, 588: 8, 629: 8, 638: 8, 639: 8, 660: 8, 665: 8, 682: 2, 692: 8, 694: 8, 724: 8, 748: 8, 786: 8, 790: 8, 792: 8, 797: 8, 798: 8, 800: 8, 801: 8, 802: 8, 803: 8, 812: 8, 813: 8, 814: 8, 815: 8, 827: 8, 828: 8, 829: 8, 833: 8, 834: 8, 835: 8, 836: 8, 843: 8, 847: 8, 848: 8, 854: 8, 860: 8, 863: 8, 879: 8, 884: 8, 906: 8, 944: 8, 951: 8, 965: 8, 973: 8, 985: 8, 1004: 8, 1020: 8, 1023: 8, 1028: 8, 1031: 8, 1036: 8, 1037: 8, 1040: 8, 1048: 8, 1052: 8, 1058: 8, 1074: 8, 1076: 8, 1098: 8, 1107: 8, 1141: 8, 1168: 8, 1178: 8, 1184: 8, 1189: 8, 1192: 8, 1193: 8, 1211: 8, 1215: 8, 1217: 8, 1246: 8, 1269: 8, 1274: 8, 1278: 8, 1297: 8, 1298: 8, 1319: 8, 1322: 8, 1542: 2, 1590: 8, 1687: 8, 1792: 8, 1798: 3, 1799: 8, 1810: 8, 1813: 8, 1824: 8, 1832: 8, 1840: 8, 1856: 8, 1858: 8, 1859: 8, 1862: 8, 1872: 8, 1879: 8, 1888: 8, 1892: 8, 1927: 8, 1937: 8, 1953: 8, 1968: 8, 1988: 8, 2000: 8, 2001: 8, 2004: 8, 2012: 8, 2015: 8, 2016: 8, 2017: 8, 2024: 8, 2027: 8
  },
  {
    85: 8, 140: 8, 213: 8, 287: 5, 289: 8, 290: 8, 291: 8, 301: 8, 307: 8, 309: 8, 324: 8, 327: 8, 330: 8, 337: 8, 356: 8, 371: 8, 384: 8, 385: 3, 410: 8, 418: 8, 432: 8, 450: 8, 482: 8, 496: 8, 508: 8, 522: 8, 536: 8, 537:      8, 544: 8, 546: 8, 547: 8, 576: 8, 577: 8, 578: 8, 588: 8, 629: 8, 638: 8, 639: 8, 660: 8, 665: 8, 682: 2, 692: 8, 694: 8, 724: 8, 748: 8, 786: 8, 790: 8, 792: 8, 797: 8, 798: 8, 800: 8, 801: 8, 802: 8, 803: 8, 812: 8, 813: 8, 814:      8, 815: 8, 831: 8, 833: 8, 834: 8, 835: 8, 836: 8, 843: 8, 847: 8, 848: 8, 854: 8, 860: 8, 863: 8, 879: 8, 884: 8, 906: 8, 944: 8, 951: 8, 965: 8, 973: 8, 985: 8, 1004: 8, 1020: 8, 1023: 8, 1028: 8, 1031: 8, 1036: 8, 1037: 8, 1040:      8, 1048: 8, 1052: 8, 1058: 8, 1074: 8, 1076: 8, 1098: 8, 1107: 8, 1141: 8, 1178: 8, 1184: 8, 1189: 8, 1193: 8, 1211: 8, 1215: 8, 1217: 8, 1246: 8, 1274: 8, 1278: 8, 1297: 8, 1319: 8, 1322: 8, 1542: 2, 1798: 3, 1824: 8, 1832: 8
  }],
}
"""

"""
# BYD ATTO 3 FW version query. Without this the FW_Query config will not work in Fingerprints
BYD_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_ECU_SOFTWARE_VERSION_NUMBER)
BYD_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_ECU_SOFTWARE_VERSION_NUMBER)

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [BYD_VERSION_REQUEST],
      [BYD_VERSION_RESPONSE],
      bus=0,
    ),
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    ),
  ],
  extra_ecus=[
    (Ecu.eps, 0x374, None),  # EPS
    (Ecu.abs, 0x3b0, None),  # ABS/VSA
    (Ecu.fwdRadar, 0x11f, None),  # Forward Radar
    (Ecu.fwdCamera, 0x242, None),  # Forward Camera
    (Ecu.hud, 0x294, None),  # hud
  ],
)
"""

#This file is located in the /opendbc/dbc/ folder
# DBC = {
#   CAR.BYD_ATTO3: dbc_dict('byd_general_pt', None),
# }
DBC = CAR.create_dbc_map()
