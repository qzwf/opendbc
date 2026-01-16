from opendbc.can.packer import CANPacker
from opendbc.car import structs
from opendbc.car.byd.values import CAR, CarControllerParams
from opendbc.car.byd import bydcan
from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
    def __init__(self, dbc_name, CP, VM):
        super().__init__(dbc_name, CP, VM)
        self.CP = CP
        self.packer = CANPacker(dbc_name)
        self.params = CarControllerParams(CP)

        # Control state tracking
        self.steer_idx = 0
        self.acc_idx = 0
        self.lkas_idx = 0

        # Apply counters for message sequencing
        self.apply_steer_last = 0
        self.steer_req_last = False

        # Longitudinal control state
        self.acc_cmd_last = 0
        self.acc_active_last = False

    def update(self, CC, CS, now_nanos):
        actuators = CC.actuators
        hud_control = CC.hudControl
        pcm_cancel_cmd = CC.cruiseControl.cancel

        can_sends = []

        # === STEERING CONTROL ===
        if self.CP.steerControlType == structs.CarParams.SteerControlType.torque:
            new_steer = int(round(actuators.steer * self.params.STEER_MAX))
            apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last,
                                                         CS.steering_torque, self.params)
        else:
            apply_steer = 0

        # Only send steering commands when openpilot is engaged
        steer_req = CC.enabled and not CS.steer_error

        # Generate steering torque message
        if self.CP.carFingerprint == CAR.BYD_ATTO3:
            can_sends.append(bydcan.create_steering_control(
                self.packer, apply_steer, steer_req, self.steer_idx))
            self.steer_idx += 1

        # === LONGITUDINAL CONTROL ===
        if self.CP.openpilotLongitudinalControl:
            # ACC command generation
            acc_cmd = 0
            if CC.enabled and not pcm_cancel_cmd:
                # Convert desired acceleration to ACC command
                acc_cmd = int(round(actuators.accel * 100))  # Scale to ACC units
                acc_cmd = max(-100, min(100, acc_cmd))  # Limit to valid range

            # Generate ACC command message
            can_sends.append(bydcan.create_acc_control(
                self.packer, acc_cmd, CC.enabled, self.acc_idx))
            self.acc_idx += 1

        # === HUD CONTROL ===
        # LKAS HUD status
        lkas_hud_active = CC.enabled and steer_req
        can_sends.append(bydcan.create_lkas_hud(
            self.packer, lkas_hud_active, hud_control.leftLaneVisible,
            hud_control.rightLaneVisible, self.lkas_idx))
        self.lkas_idx += 1

        # ACC HUD status
        if self.CP.openpilotLongitudinalControl:
            acc_hud_active = CC.enabled and CC.cruiseControl.enabled
            set_speed = hud_control.setSpeed if hud_control.setSpeed > 0 else CS.cruise_speed
            can_sends.append(bydcan.create_acc_hud(
                self.packer, acc_hud_active, set_speed, hud_control.leadVisible, self.acc_idx))

        # Store last values
        self.apply_steer_last = apply_steer
        self.steer_req_last = steer_req
        self.acc_cmd_last = acc_cmd if 'acc_cmd' in locals() else 0
        self.acc_active_last = CC.enabled if self.CP.openpilotLongitudinalControl else False

        return can_sends


def apply_driver_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, params):
    """
    Apply driver intervention and rate limiting to steering torque commands
    Based on BYD-specific parameters and safety limits
    """

    # Rate limiting - prevent sudden torque changes
    max_delta_up = params.STEER_DELTA_UP
    max_delta_down = params.STEER_DELTA_DOWN

    apply_torque = max(apply_torque_last - max_delta_down,
                      min(apply_torque_last + max_delta_up, apply_torque))

    # Driver override detection and torque limiting
    if abs(driver_torque) > params.STEER_DRIVER_ALLOWANCE:
        # Reduce allowed torque when driver is steering
        max_torque = max(0, params.STEER_MAX -
                        (abs(driver_torque) - params.STEER_DRIVER_ALLOWANCE) *
                        params.STEER_DRIVER_MULTIPLIER)
        apply_torque = max(-max_torque, min(max_torque, apply_torque))
    else:
        # Full torque available when driver is not intervening
        apply_torque = max(-params.STEER_MAX, min(params.STEER_MAX, apply_torque))

    return int(round(apply_torque))
