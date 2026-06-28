from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.byd.values import CAR, CarControllerParams
from opendbc.car.byd import bydcan
from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
    def __init__(self, dbc_names, CP):
        super().__init__(dbc_names, CP)
        self.CP = CP
        self.packer = CANPacker(dbc_names[Bus.pt])
        self.params = CarControllerParams(CP)

        self.steer_idx = 0
        self.acc_idx = 0
        self.lkas_idx = 0

        self.apply_steer_last = 0
        self.steer_req_last = False

        self.acc_cmd_last = 0

    def update(self, CC, CS, now_nanos):
        actuators = CC.actuators
        hud_control = CC.hudControl
        pcm_cancel_cmd = CC.cruiseControl.cancel

        can_sends = []

        # === STEERING CONTROL ===
        new_steer = int(round(actuators.torque * self.params.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(
            new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

        steer_req = CC.latActive

        # Only send when actively controlling — avoids fighting the stock camera LKAS on bus 2
        if self.CP.carFingerprint == CAR.BYD_ATTO3 and CC.latActive:
            can_sends.append(bydcan.create_steering_control(
                self.packer, apply_steer, steer_req, self.steer_idx))
            self.steer_idx += 1

        # === LONGITUDINAL CONTROL ===
        acc_cmd = 0
        if self.CP.openpilotLongitudinalControl:
            if CC.enabled and not pcm_cancel_cmd:
                acc_cmd = int(round(actuators.accel * 100))
                acc_cmd = max(-100, min(100, acc_cmd))

            can_sends.append(bydcan.create_acc_control(
                self.packer, acc_cmd, CC.enabled, self.acc_idx))
            self.acc_idx += 1

        # === HUD CONTROL ===
        lkas_hud_active = CC.latActive
        can_sends.append(bydcan.create_lkas_hud(
            self.packer, lkas_hud_active, hud_control.leftLaneVisible,
            hud_control.rightLaneVisible, self.lkas_idx))
        self.lkas_idx += 1

        if self.CP.openpilotLongitudinalControl:
            acc_hud_active = CC.enabled
            set_speed = hud_control.setSpeed if hud_control.setSpeed > 0 else CS.out.cruiseState.speed
            can_sends.append(bydcan.create_acc_hud(
                self.packer, acc_hud_active, set_speed, hud_control.leadVisible, self.acc_idx))

        self.apply_steer_last = apply_steer
        self.steer_req_last = steer_req
        self.acc_cmd_last = acc_cmd

        new_actuators = actuators.as_builder()
        new_actuators.torque = apply_steer / self.params.STEER_MAX
        new_actuators.torqueOutputCan = apply_steer

        return new_actuators, can_sends


def apply_driver_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, params):
    apply_torque = max(apply_torque_last - params.STEER_DELTA_DOWN,
                       min(apply_torque_last + params.STEER_DELTA_UP, apply_torque))

    if abs(driver_torque) > params.STEER_DRIVER_ALLOWANCE:
        max_torque = max(0, params.STEER_MAX -
                         (abs(driver_torque) - params.STEER_DRIVER_ALLOWANCE) *
                         params.STEER_DRIVER_MULTIPLIER)
        apply_torque = max(-max_torque, min(max_torque, apply_torque))
    else:
        apply_torque = max(-params.STEER_MAX, min(params.STEER_MAX, apply_torque))

    return int(round(apply_torque))
