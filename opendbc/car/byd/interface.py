from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.carcontroller import CarController

NetworkLocation = structs.CarParams.NetworkLocation

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter


class CarInterface(CarInterfaceBase):
    CarState = CarState
    CarController = CarController

    @staticmethod
    def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
        ret.brand = "byd"
        # SAFETY_BYD: relay engaged, forwards all camera messages except STEERING_MODULE_ADAS
        # when controls_allowed (OpenPilot steering active). Camera ADAS/ACC/AEB always pass
        # through — car retains priority. OpenPilot sends steering on bus 0 directly to EPS.
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]
        ret.radarUnavailable = True

        # BYD EPS receives absolute steering wheel angle targets in STEERING_MODULE_ADAS.
        # Angle control sends actuators.steeringAngleDeg directly, which matches the EPS protocol.
        # Torque control sent small corrections that fought every curve (EPS steered toward 0).
        ret.steerControlType = structs.CarParams.SteerControlType.angle
        ret.steerActuatorDelay = 0.1
        ret.steerLimitTimer = 0.4

        ret.longitudinalTuning.kpBP = [0., 35.]
        ret.longitudinalTuning.kpV = [1.2, 0.5]
        ret.longitudinalTuning.kiBP = [0., 35.]
        ret.longitudinalTuning.kiV = [0.18, 0.12]

        ret.networkLocation = NetworkLocation.fwdCamera

        ret.openpilotLongitudinalControl = False
        ret.pcmCruise = True
        ret.minEnableSpeed = -1
        ret.minSteerSpeed = 0.

        return ret
