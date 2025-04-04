from collections import namedtuple

from opendbc.car import DbcDict, PlatformConfig, Platforms, CarSpecs, structs, Bus
from opendbc.car.structs import CarParams
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.docs_definitions import CarDocs, CarParts, CarHarness, SupportType
from dataclasses import dataclass, field

# Multiplier used to scale HUD values for display.
# The value 0.718 is a calibration factor that converts raw CAN signal values
# to appropriate units for the vehicle's head-up display.
HUD_MULTIPLIER = 0.718

Ecu = CarParams.Ecu
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])


# Class containing parameters for BYD car control.
# Controls various aspects of steering and longitudinal control,
# including:
# - Steering torque limits and rates
# - Driver intervention thresholds
# - Control loop timing
class CarControllerParams:
    STEER_MAX = 300
    STEER_DELTA_UP = 17
    STEER_DELTA_DOWN = 17

    STEER_DRIVER_ALLOWANCE = 68
    STEER_DRIVER_MULTIPLIER = 3
    STEER_DRIVER_FACTOR = 1
    STEER_ERROR_MAX = 50
    # Steer torque clip = STEER_MAX - (DriverTorque - STEER_DRIVER_ALLOWANCE) * STEER_DRIVER_MULTIPLIER (Only work when DriverTorque > STEER_DRIVER_ALLOWANCE)
    # So DriverTorque(max) = STEER_MAX / STEER_DRIVER_MULTIPLIER + STEER_DRIVER_ALLOWANCE = 300/3+68 = 168
    # i.e. when drivertorque > 168, new_steer will be cliped to 0

    STEER_STEP = 2  # 2=50 Hz

    def __init__(self, CP):
        pass


@dataclass
class BYDCarDocs(CarDocs):
    package: str = "All"
    car_parts: CarParts = field(
        default_factory=CarParts.common([CarHarness.custom]))


@ dataclass
class BYDPlatformConfig(PlatformConfig):
    dbc_dict: DbcDict = field(default_factory=lambda: {
        Bus.pt: 'byd_general_pt',
        None:None})


class CAR(Platforms):
    BYD_ATTO3 = BYDPlatformConfig(
        [
            # The year has to be 4 digits followed by hyphen and 4 digits
            BYDCarDocs("BYD ATTO3 Electric 2022-24",
                       support_type=SupportType.COMMUNITY)
        ],
        CarSpecs(mass=1750, wheelbase=2.72, steerRatio=14.8,
                 tireStiffnessFactor=0.7983),
    )




class CanBus:
    ESC = 0
    MRR = 1
    MPC = 2
    LOOPBACK = 128


BUTTONS = [
    Button(structs.CarState.ButtonEvent.Type.leftBlinker,
           "STALKS", "LeftIndicator", [0x01]),
    Button(structs.CarState.ButtonEvent.Type.rightBlinker,
           "STALKS", "RightIndicator", [0x01]),
    Button(structs.CarState.ButtonEvent.Type.accelCruise,
           "PCM_BUTTONS", "BTN_AccUpDown_Cmd", [0x02]),
    Button(structs.CarState.ButtonEvent.Type.decelCruise,
           "PCM_BUTTONS", "BTN_AccUpDown_Cmd", [0x03]),
    Button(structs.CarState.ButtonEvent.Type.cancel,
           "PCM_BUTTONS", "BTN_AccCancel", [0x01]),
]

# Firmware query configuration for BYD vehicles
FW_QUERY_CONFIG = FwQueryConfig(
    requests=[
        Request(
            [StdQueries.UDS_VERSION_REQUEST],
            [StdQueries.UDS_VERSION_RESPONSE],
            bus=0,
        ),
    ],
    extra_ecus=[
        # All known ECUs translated from the DBC file
        (Ecu.unknown, 0x1E2, None),
        (Ecu.unknown, 0x32D, None),  # ACC_HUD_ADAS
        (Ecu.unknown, 0x316, None),  # LKAS_HUD_ADAS
        (Ecu.unknown, 0x11F, None),  # STEER Angle
    ]
)

DBC = CAR.create_dbc_map()
