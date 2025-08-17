from opendbc.car import structs
CarParams = structs.CarParams
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.byd.values import CarControllerParams, CanBus
from opendbc.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):

    @staticmethod
    def _get_params(ret: CarParams, candidate: str, fingerprint, car_fw, experimental_long, docs) -> CarParams:
        ret.carName = "byd"
        ret.safetyConfigs = [CarParams.SafetyConfig(safetyModel=CarParams.SafetyModel.byd)]
        ret.radarUnavailable = True  # BYD ATTO3 uses camera-based detection

        # Mass and geometry from values.py
        ret.mass = 1750
        ret.wheelbase = 2.72
        ret.steerRatio = 14.8
        ret.centerToFront = ret.wheelbase * 0.45  # Estimated center of mass

        # Steering actuator configuration
        ret.steerControlType = CarParams.SteerControlType.torque
        ret.steerActuatorDelay = 0.1  # Steering actuator delay in seconds
        ret.steerLimitTimer = 0.4     # Time limit for steering requests

        # Steering limits from CarControllerParams
        controller_params = CarControllerParams(ret)
        ret.steerMaxBP = [0.]         # Breakpoints for max steer
        ret.steerMaxV = [controller_params.STEER_MAX]  # Max steer values

        # Lateral tuning - PID controller for steering
        ret.lateralTuning.init('pid')
        ret.lateralTuning.pid.kiBP = [0.]
        ret.lateralTuning.pid.kpBP = [0.]
        ret.lateralTuning.pid.kpV = [0.25]    # Proportional gain
        ret.lateralTuning.pid.kiV = [0.05]    # Integral gain
        ret.lateralTuning.pid.kf = 0.00004    # Feedforward gain

        # Longitudinal control - using stock ACC when available
        if experimental_long:
            ret.experimentalLongitudinalAvailable = True
            ret.openpilotLongitudinalControl = True
            ret.longitudinalTuning.kpBP = [0., 5., 35.]
            ret.longitudinalTuning.kpV = [1.2, 0.8, 0.5]
            ret.longitudinalTuning.kiBP = [0., 35.]
            ret.longitudinalTuning.kiV = [0.18, 0.12]
            ret.longitudinalTuning.deadzoneBP = [0.]
            ret.longitudinalTuning.deadzoneV = [0.]
        else:
            ret.openpilotLongitudinalControl = False

        # Feature availability based on fingerprint
        ret.enableBsm = 1048 in fingerprint.get(CanBus.pt, {})  # BSM (Blind Spot Monitoring)
        ret.enableAutoHold = True     # Auto-hold available on BYD ATTO3

        # Safety and engagement
        ret.pcmCruise = not ret.openpilotLongitudinalControl
        ret.minEnableSpeed = -1       # Enable at any speed (including reverse)
        ret.minSteerSpeed = 0.        # Minimum speed for steering

        # Network configuration
        ret.canfd = True              # BYD ATTO3 uses CAN-FD

        return ret

    def _update(self, c):
        ret = self.CS.update(self.cp, self.cp_cam)

        # Vehicle state updates
        ret.wheelSpeeds = self.CS.get_wheel_speeds(
            self.cp.vl["WHEEL_SPEED"]["WHEELSPEED_FL"],
            self.cp.vl["WHEEL_SPEED"]["WHEELSPEED_FR"],
            self.cp.vl["WHEEL_SPEED"]["WHEELSPEED_BL"],
            self.cp.vl["WHEEL_SPEED"]["WHEELSPEED_BR"],
        )
        ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.0
        ret.vEgo, ret.aEgo = self.CS.update_speed_kf(ret.vEgoRaw)

        # Steering state
        ret.steeringAngleDeg = self.CS.steering_angle
        ret.steeringRateDeg = self.CS.steering_rate
        ret.steeringTorque = self.CS.steering_torque
        ret.steeringPressed = self.CS.steering_pressed

        # Pedal states
        ret.gas = self.CS.pedal_gas
        ret.gasPressed = self.CS.pedal_gas > 0
        ret.brake = self.CS.pedal_brake
        ret.brakePressed = self.CS.pedal_brake > 0

        # Gear and drive state
        ret.gearShifter = self.CS.gear_shifter
        ret.parkingBrake = False  # Not available on BYD ATTO3

        # Door and safety states
        ret.doorOpen = self.CS.door_open
        ret.seatbeltUnlatched = not self.CS.seatbelt_driver

        # Button events
        ret.buttonEvents = self.CS.button_events

        # ADAS engagement status
        ret.cruiseState.enabled = self.CS.acc_active
        ret.cruiseState.available = self.CS.main_on
        ret.cruiseState.speed = self.CS.cruise_speed * CV.KPH_TO_MS
        ret.cruiseState.speedCluster = ret.cruiseState.speed

        # Stock LKAS status
        ret.steerFaultTemporary = self.CS.steer_error
        ret.steerFaultPermanent = False

        # EV-specific states
        ret.chargingState = self.CS.charging_state
        ret.batteryPercent = self.CS.battery_percent

        return ret

    def apply(self, c, now_nanos):
        return self.CC.update(c, self.CS, now_nanos)
