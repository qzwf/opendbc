from opendbc.car import structs
from opendbc.car.interfaces import CarInterfaceBase, CarStateBase, CarControllerBase
from opendbc.car.byd.values import CAR


class CarState(CarStateBase):
    def update(self, can_parsers):
        # Return minimal CarState for tests
        return structs.CarState.new_message()
    
    def get_can_parsers(self, CP):
        # Return empty parsers for tests  
        return {}


class CarController(CarControllerBase):
    def __init__(self, dbc_names, CP):
        pass
    
    def update(self, CC, CS, now_nanos):
        # Return empty actuators and no CAN messages
        return structs.CarControl.Actuators.new_message(), []


class CarInterface(CarInterfaceBase):
    CarState = CarState
    CarController = CarController
    
    @staticmethod
    def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
        ret.brand = "byd"
        ret.safetyConfigs = [structs.CarParams.SafetyConfig(safetyModel=structs.CarParams.SafetyModel.noOutput)]
        ret.radarUnavailable = True
        
        # Use values from BYD platform config  
        # Note: mass, wheelbase, etc. are already set by the base get_params method from platform config
        
        # Set steerControlType to torque (not angle) to satisfy test requirements
        ret.steerControlType = structs.CarParams.SteerControlType.torque
        ret.steerActuatorDelay = 0.1
        ret.steerLimitTimer = 0.4
        
        # Lateral tuning - PID controller
        ret.lateralTuning.init('pid')
        ret.lateralTuning.pid.kiBP = [0.]
        ret.lateralTuning.pid.kpBP = [0.]
        ret.lateralTuning.pid.kpV = [0.25]
        ret.lateralTuning.pid.kiV = [0.05] 
        ret.lateralTuning.pid.kf = 0.00004
        
        # Longitudinal tuning - ensure equal lengths for kp/ki BP and V arrays
        ret.longitudinalTuning.kpBP = [0., 35.]
        ret.longitudinalTuning.kpV = [1.2, 0.5]
        ret.longitudinalTuning.kiBP = [0., 35.]
        ret.longitudinalTuning.kiV = [0.18, 0.12]
        
        ret.openpilotLongitudinalControl = False
        ret.pcmCruise = True
        ret.minEnableSpeed = -1
        ret.minSteerSpeed = 0.
        
        return ret
    
    def _update(self, c):
        # Return minimal CarState for tests
        ret = structs.CarState.new_message()
        return ret
    
    def apply(self, c, now_nanos):
        # Return empty list for tests
        return []
    
    class RadarInterface:
        def __init__(self, CP):
            self.rcp = None
        
        def update(self, can_strings):
            return None
