from collections import namedtuple
from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms, structs
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.docs_definitions import CarDocs, CarParts, CarHarness, SupportType

# HUD multiplier for display calibration
HUD_MULTIPLIER = 0.718

Ecu = structs.CarParams.Ecu
ButtonType = structs.CarState.ButtonEvent.Type
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])


def dbc_dict(pt, radar):
    return {Bus.pt: pt, Bus.radar: radar}


# BYD Car Controller Parameters - tuned for ATTO3
class CarControllerParams:
    # STEER_ANGLE DBC signal: factor=0.1 deg → packer physical value = raw/10.
    # BYD EPS uses absolute steering wheel angle targets (same units as STEER_ANGLE_2 sensor).
    # Panda max_torque=1000 raw → 100 physical degrees covers urban curves up to ~50 km/h.
    STEER_MAX = 100                   # degrees (physical); panda raw limit = 1000
    # Rate at 100 Hz carController; CAN sent at 50 Hz (STEER_STEP=2).
    # Per CAN msg: DELTA_UP × 2 calls = 1.0 deg = 10 raw = panda max_rate_up limit.
    # Over 250ms (12.5 CAN msgs): 12.5 × 10 = 125 raw = panda max_rt_delta limit.
    # Effective ramp: 50 deg/s — reaches 30° in 0.6s, 50° in 1.0s.
    STEER_DELTA_UP = 0.5              # degrees/call → 50 deg/s ramp rate (10 raw/CAN-msg)
    STEER_DELTA_DOWN = 1.0            # degrees/call → 100 deg/s release (20 raw/CAN-msg, panda allows 20)

    # Driver intervention thresholds (DRIVER_EPS_TORQUE raw units, 0–255)
    STEER_DRIVER_ALLOWANCE = 80       # observed max ~52 during normal turns; threshold for override
    STEER_DRIVER_MULTIPLIER = 1       # reduction factor above allowance
    STEER_DRIVER_FACTOR = 1           # additional scaling factor
    STEER_ERROR_MAX = 350             # not actively used but raised to avoid spurious faults

    # Control timing - 50Hz update rate
    STEER_STEP = 2

    def __init__(self, CP):
        pass


@dataclass
class BYDCarDocs(CarDocs):
    package: str = "All"
    car_parts: CarParts = field(default_factory=lambda: CarParts.common([CarHarness.custom]))


@dataclass
class BYDPlatformConfig(PlatformConfig):
    dbc_dict: dict = field(default_factory=lambda: {
        Bus.pt: 'byd_general',   # bus 0: car-side chassis CAN
        Bus.cam: 'byd_general',  # bus 2: camera-side chassis CAN
    })


# BYD Vehicle Models
class CAR(Platforms):
    BYD_ATTO3 = BYDPlatformConfig(
        [
            BYDCarDocs("BYD ATTO3 2022-24", support_type=SupportType.COMMUNITY)
        ],
        CarSpecs(
            mass=1750,               # Vehicle mass in kg
            wheelbase=2.72,          # Wheelbase in meters
            steerRatio=14.8,         # Steering ratio
            tireStiffnessFactor=0.7983  # Tire stiffness factor
        ),
    )


# CAN Bus Configuration
# Bus 0: Car-side chassis CAN (read car ECU signals from here)
# Bus 1: Private CAN (camera image/radar output, CAN-FD frames)
# Bus 2: Camera-side chassis CAN (camera sends ADAS messages here — send our overrides here)
class CanBus:
    pt = 0      # Car-side chassis CAN (receive)
    cam = 2     # Camera-side chassis CAN (send ADAS commands to compete with camera)
    radar = 1   # Private CAN (camera image/radar data)


# Button configurations for steering wheel controls
BUTTONS = [
    Button(ButtonType.leftBlinker, "STALKS", "LEFT_BLINKER", [1]),
    Button(ButtonType.rightBlinker, "STALKS", "RIGHT_BLINKER", [1]),
    Button(ButtonType.accelCruise, "PCM_BUTTONS", "RES_BTN", [1]),
    Button(ButtonType.decelCruise, "PCM_BUTTONS", "SET_BTN", [1]),
    Button(ButtonType.lkas, "PCM_BUTTONS", "LKAS_ON_BTN", [1]),
    Button(ButtonType.gapAdjustCruise, "PCM_BUTTONS", "DEC_DISTANCE_BTN", [1]),
    Button(ButtonType.gapAdjustCruise, "PCM_BUTTONS", "INC_DISTANCE_BTN", [1]),
]

# Comprehensive Firmware Query Configuration based on DBC analysis
FW_QUERY_CONFIG = FwQueryConfig(
    requests=[
        # Standard UDS diagnostic requests on primary bus
        Request(
            [StdQueries.UDS_VERSION_REQUEST],
            [StdQueries.UDS_VERSION_RESPONSE],
            bus=0,
        ),
        # Extended CAN messages on camera bus
        Request(
            [StdQueries.UDS_VERSION_REQUEST],
            [StdQueries.UDS_VERSION_RESPONSE],
            bus=1,
        ),
    ],
    extra_ecus=[
        # Critical ADAS ECUs (from DBC analysis)
        (Ecu.adas, 0x1e2, None),      # STEERING_MODULE_ADAS (482) - Key for LKAS
        (Ecu.adas, 0x316, None),      # LKAS_HUD_ADAS (790) - LKAS display
        (Ecu.adas, 0x32d, None),      # ACC_HUD_ADAS (813) - ACC display
        (Ecu.adas, 0x32e, None),      # ACC_CMD (814) - ACC control
        (Ecu.adas, 0x2b4, None),      # ADAS2 (692)
        (Ecu.adas, 0x32f, None),      # ADAS3 (815)
        (Ecu.adas, 0x34b, None),      # ADAS4 (843)
        (Ecu.adas, 0x374, None),      # ADAS5 (884)
        (Ecu.adas, 0x432, None),      # ADAS6 (1074)
        (Ecu.adas, 0x418, None),      # BSM (1048)

        # Steering System ECUs
        (Ecu.eps, 0x1fc, None),       # STEERING_TORQUE (508)
        (Ecu.eps, 0x11f, None),       # STEER_MODULE_2 (287)

        # EV-Specific ECUs (BYD ATTO3 is electric)
        (Ecu.hybrid, 0x320, None),   # Battery Management System

        # Vehicle State & Control ECUs
        (Ecu.transmission, 0x242, None),  # DRIVE_STATE (578)
        (Ecu.body, 0x3b0, None),      # PCM_BUTTONS (944)
        (Ecu.body, 0x133, None),      # STALKS (307)
        (Ecu.body, 0x294, None),      # METER_CLUSTER (660)
        (Ecu.body, 0x342, None),      # PEDAL (834)
        (Ecu.body, 0x220, None),      # PEDAL_PRESSED (544)

        # Motion & Speed ECUs
        (Ecu.abs, 0x122, None),       # WHEEL_SPEED (290)
        (Ecu.abs, 0x1f0, None),       # WHEELSPEED_CLEAN (496)

        # Gateway & Communication
        (Ecu.gateway, 0x511, None),   # CAN Gateway
    ]
)

# DBC file mapping
DBC = CAR.create_dbc_map()
