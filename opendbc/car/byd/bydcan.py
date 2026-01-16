from opendbc.car.byd.values import CanBus

# BYD CAN message checksum implementation
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


def create_steering_control(packer, apply_steer, steer_req, idx):
    """
    Create steering torque control message for BYD ATTO3
    Based on STEERING_MODULE_ADAS message (482)
    """

    # Steering request flags
    steer_req_active = 1 if steer_req else 0
    steer_req_active_low = 0 if steer_req else 1  # Inverted logic

    # Fixed values based on DBC analysis
    set_me_ff = 0xFF
    set_me_f = 0xF
    set_me_xe = 0xE
    set_me_x01 = 0x01
    set_me_1_1 = 1
    set_me_1_2 = 1

    values = {
        "STEER_ANGLE": apply_steer * 10,  # Convert to DBC units (0.1 deg)
        "STEER_REQ": steer_req_active,
        "STEER_REQ_ACTIVE_LOW": steer_req_active_low,
        "SET_ME_FF": set_me_ff,
        "SET_ME_F": set_me_f,
        "SET_ME_XE": set_me_xe,
        "SET_ME_X01": set_me_x01,
        "SET_ME_1_1": set_me_1_1,
        "SET_ME_1_2": set_me_1_2,
        "COUNTER": idx % 16,
        "CHECKSUM": 0,  # Temporary, will be calculated below
    }

    # Create message with temporary checksum
    msg = packer.make_can_msg("STEERING_MODULE_ADAS", CanBus.pt, values)

    # Calculate and set proper BYD checksum
    checksum = byd_checksum(CHECKSUM_KEY, msg[1])
    values["CHECKSUM"] = checksum

    return packer.make_can_msg("STEERING_MODULE_ADAS", CanBus.pt, values)


def create_acc_control(packer, acc_cmd, acc_enabled, idx):
    """
    Create ACC longitudinal control message
    Based on ACC_CMD message (814)
    """

    # ACC command scaling and limiting
    accel_cmd = max(-100, min(100, acc_cmd))  # Limit to valid range

    # ACC control flags
    acc_on_1 = 1 if acc_enabled else 0
    acc_on_2 = 1 if acc_enabled else 0
    cmd_req_active_low = 0 if acc_enabled else 1  # Inverted logic
    acc_controllable_and_on = 1 if acc_enabled else 0
    acc_req_not_standstill = 1 if abs(accel_cmd) > 0 else 0

    # Fixed values from DBC analysis
    set_me_25_1 = 0x25
    set_me_25_2 = 0x25
    set_me_xf = 0xF
    set_me_x8 = 0x8
    set_me_1 = 1

    values = {
        "ACCEL_CMD": accel_cmd + 100,  # Offset for DBC encoding
        "ACC_ON_1": acc_on_1,
        "ACC_ON_2": acc_on_2,
        "CMD_REQ_ACTIVE_LOW": cmd_req_active_low,
        "ACC_CONTROLLABLE_AND_ON": acc_controllable_and_on,
        "ACC_REQ_NOT_STANDSTILL": acc_req_not_standstill,
        "SET_ME_25_1": set_me_25_1,
        "SET_ME_25_2": set_me_25_2,
        "SET_ME_XF": set_me_xf,
        "SET_ME_X8": set_me_x8,
        "SET_ME_1": set_me_1,
        "ACCEL_FACTOR": 10,  # Default acceleration factor
        "DECEL_FACTOR": 10,  # Default deceleration factor
        "STANDSTILL_STATE": 0,
        "ACC_OVERRIDE_OR_STANDSTILL": 0,
        "STANDSTILL_RESUME": 0,
        "COUNTER": idx % 16,
        "CHECKSUM": 0,  # Temporary, will be calculated below
    }

    # Create message with temporary checksum
    msg = packer.make_can_msg("ACC_CMD", CanBus.pt, values)

    # Calculate and set proper BYD checksum
    checksum = byd_checksum(CHECKSUM_KEY, msg[1])
    values["CHECKSUM"] = checksum

    return packer.make_can_msg("ACC_CMD", CanBus.pt, values)


def create_lkas_hud(packer, lkas_active, left_lane, right_lane, idx):
    """
    Create LKAS HUD display message
    Based on LKAS_HUD_ADAS message (790)
    """

    # LKAS active indicators (using inverted logic as per DBC)
    steer_active_active_low = 0 if lkas_active else 1
    steer_active_1_1 = 1 if lkas_active else 0
    steer_active_1_2 = 1 if lkas_active else 0
    steer_active_1_3 = 1 if lkas_active else 0

    # Lane line status for HUD display
    lss_state = 0
    if left_lane and right_lane:
        lss_state = 3  # Both lanes visible
    elif left_lane:
        lss_state = 1  # Left lane only
    elif right_lane:
        lss_state = 2  # Right lane only

    # Fixed values from DBC
    set_me_xff = 0xFF
    set_me_x5f = 0x5F
    set_me_1_2 = 1

    values = {
        "STEER_ACTIVE_ACTIVE_LOW": steer_active_active_low,
        "STEER_ACTIVE_1_1": steer_active_1_1,
        "STEER_ACTIVE_1_2": steer_active_1_2,
        "STEER_ACTIVE_1_3": steer_active_1_3,
        "LSS_STATE": lss_state,
        "SET_ME_XFF": set_me_xff,
        "SET_ME_X5F": set_me_x5f,
        "SET_ME_1_2": set_me_1_2,
        "SETTINGS": idx % 16,  # Use counter for settings
        "HAND_ON_WHEEL_WARNING": 0,  # No warning by default
        "HMA": 0,  # High beam assist off
        "PT2": 0,
        "PT3": 0,
        "PT4": 0,
        "PT5": 0,
        "TSR": 0,  # Traffic sign recognition
        "COUNTER": idx % 16,
        "CHECKSUM": 0,  # Temporary, will be calculated below
    }

    # Create message with temporary checksum
    msg = packer.make_can_msg("LKAS_HUD_ADAS", CanBus.pt, values)

    # Calculate and set proper BYD checksum
    checksum = byd_checksum(CHECKSUM_KEY, msg[1])
    values["CHECKSUM"] = checksum

    return packer.make_can_msg("LKAS_HUD_ADAS", CanBus.pt, values)


def create_acc_hud(packer, acc_active, set_speed, lead_visible, idx):
    """
    Create ACC HUD display message
    Based on ACC_HUD_ADAS message (813)
    """

    # ACC status indicators
    acc_on1 = 1 if acc_active else 0
    acc_on2 = 1 if acc_active else 0

    # Speed conversion (km/h to DBC units)
    set_speed_dbc = int(set_speed * 2) if set_speed > 0 else 0  # 0.5 km/h units
    set_speed_dbc = max(0, min(255, set_speed_dbc))  # Limit to valid range

    # Distance setting (default to middle setting)
    set_distance = 2  # "2bar" setting

    # Fixed values from DBC
    set_me_xf = 0xF
    set_me_xff = 0xFF

    values = {
        "ACC_ON1": acc_on1,
        "ACC_ON2": acc_on2,
        "SET_SPEED": set_speed_dbc,
        "SET_DISTANCE": set_distance,
        "SET_ME_XF": set_me_xf,
        "SET_ME_XFF": set_me_xff,
        "COUNTER": idx % 16,
        "CHECKSUM": 0,  # Temporary, will be calculated below
    }

    # Create message with temporary checksum
    msg = packer.make_can_msg("ACC_HUD_ADAS", CanBus.pt, values)

    # Calculate and set proper BYD checksum
    checksum = byd_checksum(CHECKSUM_KEY, msg[1])
    values["CHECKSUM"] = checksum

    return packer.make_can_msg("ACC_HUD_ADAS", CanBus.pt, values)


def create_steering_torque(packer, main_torque, idx):
    """
    Create main steering torque message (read from vehicle)
    Based on STEERING_TORQUE message (508) - typically read-only
    """

    values = {
        "MAIN_TORQUE": int(main_torque * 10),  # Convert to DBC units (0.1 Nm)
        "COUNTER": idx % 16,
        "CHECKSUM": 0,  # Temporary, will be calculated below
    }

    # Create message with temporary checksum
    msg = packer.make_can_msg("STEERING_TORQUE", CanBus.pt, values)

    # Calculate and set proper BYD checksum
    checksum = byd_checksum(CHECKSUM_KEY, msg[1])
    values["CHECKSUM"] = checksum

    return packer.make_can_msg("STEERING_TORQUE", CanBus.pt, values)
