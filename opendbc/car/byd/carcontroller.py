import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.byd import bydcan
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
    def __init__(self, dbc_names, CP):
        super().__init__(dbc_names, CP)
        self.CP = CP
        self.packer = CANPacker(dbc_names[Bus.pt])
        self.params = CarControllerParams(CP)

        self.apply_angle_last = 0.0
        self.acc_idx = 0

    def update(self, CC, CS, now_nanos):
        actuators = CC.actuators
        hud_control = CC.hudControl
        pcm_cancel_cmd = CC.cruiseControl.cancel

        can_sends = []

        # === STEERING ===
        # The EPS is a position servo: STEER_ANGLE is an absolute wheel angle target.
        # The command must therefore stay anchored to the measured angle at all times —
        # a limiter that only tracks its own previous output can ratchet away from the
        # wheel, saturate at the clamp and get every frame rejected by panda.
        if self.frame % self.params.STEER_STEP == 0:
            apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last,
                                                       CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                       CC.latActive, self.params.ANGLE_LIMITS)

            # Hand control back to the driver rather than fighting them. DRIVER_EPS_TORQUE is
            # an unsigned magnitude from the column sensor, so this is a pure override test.
            if CS.out.steeringTorque > self.params.STEER_DRIVER_ALLOWANCE:
                apply_angle = CS.out.steeringAngleDeg

            # Windup guard: never let the command drift outside a fixed window around the
            # measured angle. Makes the saturation failure mode structurally impossible.
            apply_angle = float(np.clip(apply_angle,
                                        CS.out.steeringAngleDeg - self.params.MAX_ANGLE_ERROR,
                                        CS.out.steeringAngleDeg + self.params.MAX_ANGLE_ERROR))

            self.apply_angle_last = apply_angle

            # Always transmit, even when disengaged — the EPS expects a continuous stream, and
            # panda checks that the inactive command tracks the measured angle.
            can_sends.append(bydcan.create_steering_control(self.packer, apply_angle, CC.latActive,
                                                            CS.out.standstill, self.frame // self.params.STEER_STEP))

            # We own this ID now: the camera's copy is blocked from forwarding, so the cluster
            # only sees ours. Non-LKAS fields are mirrored from the camera.
            can_sends.append(bydcan.create_lkas_hud(self.packer, CC.latActive, CS.out.steeringPressed,
                                                    CS.lkas_hud, self.frame // self.params.STEER_STEP))

        # === LONGITUDINAL ===
        if self.CP.openpilotLongitudinalControl:
            if self.frame % self.params.STEER_STEP == 0:
                accel = float(np.clip(actuators.accel, self.params.ACCEL_MIN, self.params.ACCEL_MAX))
                if not CC.longActive or pcm_cancel_cmd:
                    accel = 0.0

                can_sends.append(bydcan.create_acc_control(self.packer, accel,
                                                           CC.longActive and not pcm_cancel_cmd, self.acc_idx))

                set_speed = hud_control.setSpeed if hud_control.setSpeed > 0 else CS.out.cruiseState.speed
                can_sends.append(bydcan.create_acc_hud(self.packer, CC.enabled, set_speed * 3.6,
                                                       hud_control.leadVisible, self.acc_idx))
                self.acc_idx += 1

        new_actuators = actuators.as_builder()
        new_actuators.steeringAngleDeg = self.apply_angle_last

        self.frame += 1
        return new_actuators, can_sends
