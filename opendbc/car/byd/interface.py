#!/usr/bin/env python3

from opendbc.car import get_safety_config, structs, STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.byd.values import CAR, HUD_MULTIPLIER
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.interfaces import CarInterfaceBase

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
SafetyModel = structs.CarParams.SafetyModel

TransmissionType = structs.CarParams.TransmissionType  # GR QZWF


class CarInterface(CarInterfaceBase):
    CarState = CarState
    CarController = CarController

    @staticmethod
    def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
        ret.brand = "byd"

        # Use proper safety configuration for BYD vehicles
        # For now, using allOutput for development - should be replaced with proper BYD safety model
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.allOutput)]

        ret.dashcamOnly = candidate not in (CAR.BYD_ATTO3,)

        ret.safetyConfigs[0].safetyParam = 1
        ret.transmissionType = TransmissionType.automatic
        ret.radarUnavailable = True
        ret.enableDsu = False                  # driving support unit

        # Lateral MPC cost on steering rate, higher value = sharper turn
        ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
        ret.steerControlType = structs.CarParams.SteerControlType.angle
        # Measured at 0.4s, however in torqued.py, line 55, code will add 0.2
        ret.steerActuatorDelay = 0.2          # Steering wheel actuator delay in seconds

        ret.openpilotLongitudinalControl = True

        if candidate == CAR.BYD_ATTO3:
            ret.wheelbase = 2.72
            ret.steerRatio = 16.0
            ret.centerToFront = ret.wheelbase * 0.44
            tire_stiffness_factor = 0.9871
            ret.mass = 2090. + STD_CARGO_KG
            # the HUD odo is exactly 1 to 1 with gps speed
            ret.wheelSpeedFactor = HUD_MULTIPLIER

            # currently not in use, byd is using stock long
            ret.longitudinalTuning.kpBP = [0., 5., 20.]
            ret.longitudinalTuning.kpV = [1.5, 1.3, 1.0]
            ret.longitudinalActuatorDelay = 0.4

        else:
            ret.dashcamOnly = True
            ret.safetyModel = structs.CarParams.SafetyModel.noOutput

        ret.longitudinalTuning.kiBP = [0., 5., 20.]
        ret.longitudinalTuning.kiV = [0.32, 0.23, 0.12]

        ret.minEnableSpeed = -1
        ret.enableBsm = True
        ret.stoppingDecelRate = 0.05  # reach stopping target smoothly

        ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
        ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(
            ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

        ret.minSteerSpeed = 2 * CV.KPH_TO_MS

        ret.steerLimitTimer = 0.4

        ret.lateralTuning.init('pid')
        ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [
            [10., 40.], [10., 40.]]
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [
            [0.16, 0.33], [0.015, 0.025]]
        ret.lateralTuning.pid.kf = 0.00004

        return ret
