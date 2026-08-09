import numpy as np

from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.byd import bydcan
from opendbc.car.byd.carstate import STEER_SEQ_MASK, unpack_steer_seq
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
        self.steer_seq = 0
        self.lat_active_last = False

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

            # Transmit only while actually steering. Whenever we go quiet, panda stops
            # blocking the camera's command after ~150ms and the stock LKAS takes the wheel
            # back — so there is never a window with nobody driving the EPS. It also lets the
            # camera keep refreshing the frame template we copy.
            if CC.latActive:
                # Latch the camera's sequence on engage, then keep advancing it at the camera's
                # own rate so the stream the EPS sees carries on unbroken from the camera's.
                if not self.lat_active_last:
                    self.steer_seq = CS.steer_seq
                else:
                    self.steer_seq = (self.steer_seq + CS.steer_seq_step) & STEER_SEQ_MASK

                if self.steer_seq:
                    template = unpack_steer_seq(self.steer_seq)
                else:
                    # The camera has not steered since boot (blocked lens, LKAS never engaged),
                    # so there is no sequence to continue — and all-zero bytes make the EPS
                    # ignore the command. Fall back to the static frame bukapilot shipped for
                    # this exact car: SET_ME_X01 "must be 0x1 to steer"; SET_ME_XE 0xB while
                    # moving ("faults less, highest angle limit at high speed"), 0xE at
                    # standstill. The camera's own frames use the same 0xB/0xE nibble.
                    template = {"UNKNOWN": 0, "SET_ME_X01": 0x1,
                                "SET_ME_XE": 0xE if CS.out.standstill else 0xB}

                can_sends.append(bydcan.create_steering_control(self.packer, apply_angle,
                                                                template,
                                                                self.frame // self.params.STEER_STEP))
            self.lat_active_last = CC.latActive

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
