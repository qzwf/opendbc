from opendbc.car import structs
# from typing import dict, Any, tuple, List, Optional
from typing import Any
GearShifter = structs.CarState.GearShifter
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# Constants
CHECKSUM_KEY = 0xAF  # BYD CAN message checksum key


def byd_checksum(byte_key: int, dat: bytes) -> int:
    """Calculate BYD's CAN message checksum.

    The checksum is calculated by processing the message bytes in two parts:
    - First calculating sums of the high and low nibbles separately
    - Then applying a specific algorithm involving remainders and offsets

    Args:
        byte_key: The checksum key specific to the message type
        dat: The message data bytes to calculate checksum for

    Returns:
        The calculated checksum byte
    """
    first_bytes_sum = sum(byte >> 4 for byte in dat)
    second_bytes_sum = sum(byte & 0xF for byte in dat)
    remainder = second_bytes_sum >> 4
    second_bytes_sum += byte_key >> 4
    first_bytes_sum += byte_key & 0xF
    first_part = ((-first_bytes_sum + 0x9) & 0xF)
    second_part = ((-second_bytes_sum + 0x9) & 0xF)
    return (((first_part + (-remainder + 5)) << 4) + second_part) & 0xFF

# MPC -> Panda -> EPS


def create_steering_control(packer: Any, CP: Any, cam_msg: dict[str, Any], req_torque: int, req_prepare: int, active: bool, Counter: int) -> tuple[int, bytes]:
    """Create CAN message for steering control.

    This function constructs a CAN message to send steering control commands from
    MPC (Model Predictive Control) to the EPS (Electric Power Steering) via the Panda.

    Args:
        packer: Object that handles CAN message packing
        CP: Car Parameters object
        cam_msg: dictionary containing camera message data
        req_torque: Requested steering torque
        req_prepare: Flag indicating if LKAS should prepare
        active: Flag indicating if control is active
        Counter: Message counter for CAN bus

    Returns:
        tuple containing CAN address and message data
    """
    values = {}
    values = {s: cam_msg for s in [
        "AutoFullBeamState",
        "LeftLaneState",
        "LKAS_Config",
        "SETME2_0x1",
        "MPC_State",
        "AutoFullBeam_OnOff",
        "LKAS_Output",
        "LKAS_Active",
        "SETME3_0x0",
        "TrafficSignRecognition_OnOff",
        "SETME4_0x0",
        "SETME5_0x1",
        "RightLaneState",
        "LKAS_State",
        "SETME6_0x0",
        "TrafficSignRecognition_Result",
        "LKAS_AlarmType",
        "SETME7_0x3",
    ]}

    values["ReqHandsOnSteeringWheel"] = 0
    values["LKAS_ReqPrepare"] = req_prepare
    values["Counter"] = Counter

    if active:
        values.update({
            "LKAS_Output": req_torque,
            "LKAS_Active": 1,
            "LKAS_State": 2,
        })

    data = packer.make_can_msg("ACC_MPC_STATE", 0, values)[1]
    values["CheckSum"] = byd_checksum(CHECKSUM_KEY, data)
    return packer.make_can_msg("ACC_MPC_STATE", 0, values)

# reserved for long control


def acc_command(packer: Any, CP: Any, cam_msg: dict[str, Any], speed: float, enabled: bool) -> tuple[int, bytes]:
    """Create CAN message for adaptive cruise control commands.

    This function constructs a CAN message for the Adaptive Cruise Control (ACC) system.
    It is reserved for longitudinal control.

    Args:
        packer: Object that handles CAN message packing
        CP: Car Parameters object
        cam_msg: dictionary containing camera message data
        speed: Target vehicle speed
        enabled: Flag indicating if ACC is enabled

    Returns:
        tuple containing CAN address and message data
    """
    values = {}
    values = {s: cam_msg[s] for s in [
        "AccelCmd",
        "ComfortBandUpper",
        "ComfortBandLower",
        "JerkUpperLimit",
        "SETME1_0x1",
        "JerkLowerLimit",
        "ResumeFromStandstill",
        "StandstillState",
        "BrakeBehaviour",
        "AccReqNotStandstill",
        "AccControlActive",
        "AccOverrideOrStandstill",
        "EspBehaviour",
        "Counter",
        "SETME2_0xF",
    ]}

    data = packer.make_can_msg("ACC_CMD", 0, values)[1]
    values["CheckSum"] = byd_checksum(CHECKSUM_KEY, data)
    return packer.make_can_msg("ACC_CMD", 0, values)


# send fake torque feedback from eps to trick MPC, preventing DTC, so that safety features such as AEB still working
def create_fake_318(packer: Any, CP: Any, esc_msg: dict[str, Any], faketorque: int, laks_reqprepare: bool,
                    laks_active: bool, enabled: bool, counter: int) -> tuple[int, bytes]:
    """Create fake torque feedback message from EPS to MPC.

    This function generates a fake torque feedback message from the Electric Power Steering (EPS) to
    the Model Predictive Control (MPC). This prevents Diagnostic Trouble Codes (DTC) and ensures
    that safety features like Automatic Emergency Braking (AEB) continue to function.

    Args:
        packer: Object that handles CAN message packing
        CP: Car Parameters object
        esc_msg: dictionary containing ESC message data
        faketorque: Fake torque value to report
        laks_reqprepare: Flag indicating if LKAS is requesting preparation
        laks_active: Flag indicating if LKAS is active
        enabled: Flag indicating if control is enabled
        counter: Message counter for CAN bus

    Returns:
        tuple containing CAN address and message data
    """
    values = {}
    values = {s: esc_msg for s in [
        "LKAS_Prepared",
        "CruiseActivated",
        "TorqueFailed",
        "SETME1_0x1",
        "SteerWarning",
        "SteerError_1",
        "SteerError_2",
        "SETME2_0x0",
        "MainTorque",
        "SETME3_0x1",
        "SETME4_0x3",
        "SteerDriverTorque",
        "SETME5_0xFF",
        "SETME6_0xFFF",
    ]}

    values["ReportHandsNotOnSteeringWheel"] = 0
    values["Counter"] = counter

    if enabled:
        if laks_active:
            values.update({
                "LKAS_Prepared": 0,
                "CruiseActivated": 1,
                "MainTorque": faketorque,
            })
        elif laks_reqprepare:
            values.update({
                "LKAS_Prepared": 1,
                "CruiseActivated": 0,
                "MainTorque": 0,
            })
        else:
            values.update({
                "LKAS_Prepared": 0,
                "CruiseActivated": 0,
                "MainTorque": 0,
            })

    data = packer.make_can_msg("ACC_EPS_STATE", 2, values)[1]
    values["CheckSum"] = byd_checksum(CHECKSUM_KEY, data)
    return packer.make_can_msg("ACC_EPS_STATE", 2, values)
