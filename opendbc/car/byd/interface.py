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
        # allOutput with param=1 (PASSTHROUGH): relay engaged + forwarding enabled.
        # param=0 sets disable_forwarding=True which cuts camera<->car CAN entirely,
        # causing "MMW Radar not available" and "MVC not available" in the car's HUD.
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.allOutput, 1)]
        ret.radarUnavailable = True

        ret.steerControlType = structs.CarParams.SteerControlType.torque
        ret.steerActuatorDelay = 0.1
        ret.steerLimitTimer = 0.4

        CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

        ret.lateralParams.torqueBP = [0, 2560]
        ret.lateralParams.torqueV = [0, 2560]

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
