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
    # i.e. when drivertorque > 168, new_steer will be clipped to 0

    STEER_STEP = 2  # 2=50 Hz

    def __init__(self, CP):
        pass


@dataclass
class BYDCarDocs(CarDocs):
    package: str = "All"
    car_parts: CarParts = field(
        default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class BYDPlatformConfig(PlatformConfig):
    dbc_dict: DbcDict = field(default_factory=lambda: {
        Bus.pt: 'byd_general_pt',
        None: None})


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

# Enhanced firmware query configuration for BYD vehicles
# Designed to be comprehensive while providing fallback options
FW_QUERY_CONFIG = FwQueryConfig(
    requests=[
        # Primary UDS query for standard ECUs - essential for fingerprinting
        Request(
            [StdQueries.UDS_VERSION_REQUEST],
            [StdQueries.UDS_VERSION_RESPONSE],
            whitelist_ecus=[Ecu.engine, Ecu.abs, Ecu.eps, Ecu.hvac, Ecu.gateway, Ecu.adas],
            bus=0,
        ),
        # Query for EV-specific ECUs - battery and motor systems
        Request(
            [StdQueries.UDS_VERSION_REQUEST],
            [StdQueries.UDS_VERSION_RESPONSE],
            whitelist_ecus=[Ecu.hybrid, Ecu.electricBrakeBooster],
            bus=0,
            logging=True,  # These are for data collection, not essential for fingerprinting
        ),
    ],
    extra_ecus=[
        # Standard OBD-II ECUs (most reliable for fingerprinting)
        (Ecu.engine, 0x7E0, None),              # Engine Control Module (UDS)
        (Ecu.abs, 0x760, None),                 # ABS Control Module (UDS)
        (Ecu.eps, 0x732, None),                 # Electric Power Steering (UDS)
        (Ecu.hvac, 0x7B3, None),                # HVAC Control Module (confirmed working)
        (Ecu.hvac, 0x706, None),                # HVAC Control Module (alternative address)
        (Ecu.gateway, 0x7D0, None),             # Body Control Module/Gateway (UDS)
        
        # ADAS ECUs (critical for OpenPilot functionality)
        (Ecu.adas, 0x1E2, None),                # STEERING_MODULE_ADAS (482 decimal)
        (Ecu.adas, 0x316, None),                # LKAS_HUD_ADAS (790 decimal)
        (Ecu.adas, 0x32D, None),                # ACC_HUD_ADAS (813 decimal)
        (Ecu.adas, 0x32E, None),                # ACC_CMD (814 decimal)
        
        # EV-specific ECUs (BYD ATTO3 is electric)
        (Ecu.hybrid, 0x320, None),              # Battery Management System (800 decimal)
        (Ecu.hybrid, 0x321, None),              # Motor Controller 1 (801 decimal)
        (Ecu.hybrid, 0x322, None),              # Motor Controller 2 (802 decimal)
        (Ecu.hybrid, 0x323, None),              # Charging System (803 decimal)
        
        # Additional steering system ECUs
        (Ecu.eps, 0x1FC, None),                 # STEERING_TORQUE (508 decimal)
        (Ecu.eps, 0x11F, None),                 # STEER_MODULE_2 (287 decimal)
        
        # Other detected ECUs from CAN analysis (using electricBrakeBooster for misc)
        (Ecu.electricBrakeBooster, 0x55, None), # Address 85 decimal
        (Ecu.electricBrakeBooster, 0x8C, None), # Address 140 decimal
        (Ecu.electricBrakeBooster, 0xD5, None), # Address 213 decimal
    ]
)

DBC = CAR.create_dbc_map()
