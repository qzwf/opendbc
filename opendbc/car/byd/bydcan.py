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


def create_steering_control(packer, apply_angle, steer_req, standstill, idx):
    """
    Create the steering command for BYD ATTO3 — STEERING_MODULE_ADAS (0x1E2).

    STEER_ANGLE is an absolute steering wheel angle target (DBC factor 0.1 deg), so the
    EPS servos the wheel to this position. The SET_ME_* constants are not decorative:
    the EPS validates them and faults or ignores the command if they are wrong.
      - SET_ME_X01 must be 1 to steer, 0 when idle.
      - SET_ME_XE must be 0xB while driving and 0xE at standstill; 0xE while moving
        raises the EPS fault rate and lowers the accepted angle limit at speed.
    """

    # Only assert the fixed "armed" constants while actually requesting steering.
    if steer_req:
        set_me_x01 = 0x1
        set_me_xe = 0xE if standstill else 0xB
    else:
        set_me_x01 = 0
        set_me_xe = 0

    values = {
        "STEER_ANGLE": apply_angle,  # degrees; DBC factor 0.1 → raw = deg × 10
        "STEER_REQ": 1 if steer_req else 0,
        "STEER_REQ_ACTIVE_LOW": 0 if steer_req else 1,
        "SET_ME_FF": 0xFF,
        "SET_ME_F": 0xF,
        "SET_ME_XE": set_me_xe,
        "SET_ME_X01": set_me_x01,
        "SET_ME_1_1": 1,
        "SET_ME_1_2": 1,
        "COUNTER": idx % 16,
        "CHECKSUM": 0,  # placeholder, computed below
    }

    # Sent on bus 0, straight to the EPS. panda blocks the camera's copy from being
    # forwarded 2->0 (check_relay), so ours is the only command the EPS sees.
    msg = packer.make_can_msg("STEERING_MODULE_ADAS", CanBus.pt, values)
    values["CHECKSUM"] = byd_checksum(CHECKSUM_KEY, msg[1])

    return packer.make_can_msg("STEERING_MODULE_ADAS", CanBus.pt, values)


def create_acc_control(packer, accel, acc_enabled, idx):
    """
    Create ACC longitudinal control message — ACC_CMD (814).

    NOTE: not reachable today. openpilotLongitudinalControl is False and panda leaves
    ACC_CMD out of the TX allowlist, so this is blocked. The ACCEL_CMD scale below is
    inferred, not measured — calibrate it on the car before enabling longitudinal.
    """

    # ACCEL_CMD physical units are roughly m/s^2 * 16.67; the DBC applies the -100 offset
    accel_cmd = max(-50, min(30, int(round(accel * 16.67))))

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
        "ACCEL_CMD": accel_cmd,
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


def create_lkas_hud(packer, lkas_active, hand_on_wheel_warning, cam, idx):
    """
    Create the LKAS HUD message — LKAS_HUD_ADAS (0x316), sent on bus 0 to the cluster.

    Everything that isn't ours (lane-line state, traffic sign recognition, high beam
    assist and the PT2-PT5 / SET_ME_* passthrough fields) is mirrored from the camera's
    own copy read on bus 2. Zeroing those blanks out unrelated driver-assist icons and
    upsets the cluster, so `cam` carries the camera's last-seen values.
    """

    values = {
        "STEER_ACTIVE_ACTIVE_LOW": 0 if lkas_active else 1,
        "STEER_ACTIVE_1_1": 1 if lkas_active else 0,
        "STEER_ACTIVE_1_2": 1 if lkas_active else 0,
        "STEER_ACTIVE_1_3": 1 if lkas_active else 0,
        "HAND_ON_WHEEL_WARNING": 1 if hand_on_wheel_warning else 0,
        # camera passthrough
        "LSS_STATE": cam["LSS_STATE"],
        "SETTINGS": cam["SETTINGS"],
        "SET_ME_XFF": cam["SET_ME_XFF"],
        "SET_ME_X5F": cam["SET_ME_X5F"],
        "TSR": cam["TSR"],
        "HMA": cam["HMA"],
        "PT2": cam["PT2"],
        "PT3": cam["PT3"],
        "PT4": cam["PT4"],
        "PT5": cam["PT5"],
        "SET_ME_1_2": 1,
        "COUNTER": idx % 16,
        "CHECKSUM": 0,  # placeholder, computed below
    }

    msg = packer.make_can_msg("LKAS_HUD_ADAS", CanBus.pt, values)
    values["CHECKSUM"] = byd_checksum(CHECKSUM_KEY, msg[1])

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
