import copy
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import structs, Bus
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.numpy_fast import mean
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC


ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
    def __init__(self, CP):
        super().__init__(CP)

        can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

        self.shifter_values = can_define.dv["DRIVE_STATE"]["GEAR"]

        self.acc_hud_adas_counter = 0
        self.acc_mpc_state_counter = 0
        self.acc_cmd_counter = 0

        self.acc_active_last = False
        self.low_speed_alert = False
        self.lkas_allowed_speed = False

        self.lkas_prepared = False  # 318, EPS to OP
        self.acc_state = 0

        self.mpc_laks_output = 0
        self.mpc_laks_active = False
        self.mpc_laks_reqprepare = False

        self.cam_lkas = 0
        self.cam_acc = 0
        self.esc_eps = 0

        self.set_distance_values = can_define.dv['ACC_HUD_ADAS']['SET_DISTANCE']
        self.is_cruise_latch = False
        self.prev_angle = 0
        self.lss_state = 0
        self.lss_alert = 0
        self.tsr = 0
        self.ahb = 0
        self.passthrough = 0
        self.lka_on = 0
        self.HMA = 0
        self.pt2 = 0
        self.pt3 = 0
        self.pt4 = 0
        self.pt5 = 0
        self.lkas_rdy_btn = False
        # self.button_states = {button.event_type: False for button in BUTTONS}

    def update(self, can_parsers) -> structs.CarState:
        cp = can_parsers[Bus.pt]
        cp_cam = can_parsers[Bus.cam]

        ret = structs.CarState()

        # self.distance_button = cp.vl["PCM_BUTTONS"]["BTN_AccDistanceIncrease"]

        self.lkas_prepared = cp.vl["ACC_EPS_STATE"]["LKAS_Prepared"]

        lkas_config_isAccOn = (cp_cam.vl["ACC_MPC_STATE"]["LKAS_Config"] != 0)
        lkas_isMainSwOn = (cp.vl["PCM_BUTTONS"]["BTN_TOGGLE_ACC_OnOff"] == 1)

        lkas_hud_AccOn1 = (cp_cam.vl["ACC_HUD_ADAS"]["AccOn1"] == 1)
        self.acc_state = cp_cam.vl["ACC_HUD_ADAS"]["AccState"]

        self.tsr = cp.vl["LKAS_HUD_ADAS"]['TSR']
        self.lka_on = cp.vl["LKAS_HUD_ADAS"]['STEER_ACTIVE_ACTIVE_LOW']
        self.lkas_rdy_btn = cp_cam.vl["PCM_BUTTONS"]['LKAS_ON_BTN']
        self.abh = cp.vl["LKAS_HUD_ADAS"]['SET_ME_XFF']
        self.passthrough = cp.vl["LKAS_HUD_ADAS"]['SET_ME_X5F']
        self.HMA = cp.vl["LKAS_HUD_ADAS"]['HMA']
        self.pt2 = cp.vl["LKAS_HUD_ADAS"]['PT2']
        self.pt3 = cp.vl["LKAS_HUD_ADAS"]['PT3']
        self.pt4 = cp.vl["LKAS_HUD_ADAS"]['PT4']
        self.pt5 = cp.vl["LKAS_HUD_ADAS"]['PT5']
        self.counter_pcm_buttons = cp.vl["PCM_BUTTONS"]['COUNTER']

        # use wheels averages if you like
        # ret.wheelSpeeds = self.get_wheel_speeds(
        #     cp.vl["IPB1"]["WheelSpeed_FL"],
        #     cp.vl["IPB1"]["WheelSpeed_FR"],
        #     cp.vl["IPB1"]["WheelSpeed_RL"],
        #     cp.vl["IPB1"]["WheelSpeed_RR"],
        # )
        # speed_kph = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])

# EV irrelevant messages
        ret.brakeHoldActive = False

        ret.wheelSpeeds = self.get_wheel_speeds(
            cp.vl["WHEEL_SPEED"]['WHEELSPEED_FL'],
            cp.vl["WHEEL_SPEED"]['WHEELSPEED_FR'],
            cp.vl["WHEEL_SPEED"]['WHEELSPEED_BL'],
            # TODO: why would BR make the value wrong? Wheelspeed sensor prob?
            cp.vl["WHEEL_SPEED"]['WHEELSPEED_BL'],
        )
        ret.vEgoRaw = mean([ret.wheelSpeeds.rr, ret.wheelSpeeds.rl,
                           ret.wheelSpeeds.fr, ret.wheelSpeeds.fl])

        # unfiltered speed from CAN sensors
        ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
        ret.vEgoCluster = ret.vEgo
        ret.standstill = ret.vEgoRaw < 0.05

        # use dash speedo as speed reference
        speed_raw = int(cp.vl["CARSPEED"]["CarDisplaySpeed"])
        speed_kph = speed_raw * 0.07143644  # this constant varies with vehicles
        ret.vEgoRaw = speed_kph * CV.KPH_TO_MS  # KPH to m/s
        ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

        ret.standstill = (speed_raw == 0)

        if self.CP.minSteerSpeed > 0:
            if speed_kph > 2:
                self.lkas_allowed_speed = True
            elif speed_kph < 0.1:
                self.lkas_allowed_speed = False
        else:
            self.lkas_allowed_speed = True

        can_gear = int(cp.vl["DRIVE_STATE"]["GEAR"])
        ret.gearShifter = self.parse_gear_shifter(
            self.shifter_values.get(can_gear, None))

        # for button in BUTTONS:
        #    state = (cp.vl[button.can_addr][button.can_msg] in button.values)
        #   if self.button_states[button.event_type] != state:
        #        event = structs.CarState.ButtonEvent.new_message()
        #        event.type = button.event_type
        #        event.pressed = state
        #        self.button_events.append(event)
        #    self.button_states[button.event_type] = state

        # ret.genericToggle = bool(cp.vl["STALKS"]["HeadLight"])

        # ret.leftBlindspot = cp.vl["BSD_RADAR"]["LEFT_ROACH"] != 0
        # ret.rightBlindspot = cp.vl["BSD_RADAR"]["RIGHT_ROACH"] != 0

        ret.leftBlinker = (cp.vl["STALKS"]["LeftIndicator"] == 1)
        ret.rightBlinker = (cp.vl["STALKS"]["RightIndicator"] == 1)

        ret.steeringAngleDeg = cp.vl["EPS"]["SteeringAngle"]
        ret.steeringRateDeg = cp.vl["EPS"]["SteeringAngleRate"]

        ret.steeringTorque = cp.vl["ACC_EPS_STATE"]["SteerDriverTorque"]
        ret.steeringTorqueEps = cp.vl["ACC_EPS_STATE"]["MainTorque"]

        self.eps_state_counter = int(cp.vl["ACC_EPS_STATE"]["Counter"])

        ret.steeringPressed = abs(ret.steeringTorque) > 30

        ret.brake = int(cp.vl["PEDAL"]["BrakePedal"])

        ret.brakePressed = (ret.brake != 0)

        ret.seatbeltUnlatched = (cp.vl["BCM"]["DriverSeatBeltFasten"] != 1)

        ret.doorOpen = any([cp.vl["BCM"]["FrontLeftDoor"], cp.vl["BCM"]["FrontRightDoor"],
                            cp.vl["BCM"]["RearLeftDoor"],  cp.vl["BCM"]["RearRightDoor"]])

        ret.gas = cp.vl["PEDAL"]["AcceleratorPedal"]
        ret.gasPressed = (ret.gas != 0)

        ret.cruiseState.available = lkas_isMainSwOn and lkas_config_isAccOn and lkas_hud_AccOn1
        ret.cruiseState.enabled = (
            self.acc_state == 3) or (self.acc_state == 5)
        ret.cruiseState.standstill = ret.standstill
        ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SetSpeed"] * CV.KPH_TO_MS

        if ret.cruiseState.enabled:
            if not self.lkas_allowed_speed and self.acc_active_last:
                self.low_speed_alert = True
            else:
                self.low_speed_alert = False
        ret.lowSpeedAlert = self.low_speed_alert

        ret.steerFaultTemporary = (self.acc_state == 7)

        self.acc_active_last = ret.cruiseState.enabled

        # use to fool mpc
        self.mpc_laks_output = cp_cam.vl["ACC_MPC_STATE"]["LKAS_Output"]
        # use to fool mpc
        self.mpc_laks_reqprepare = cp_cam.vl["ACC_MPC_STATE"]["LKAS_ReqPrepare"] != 0
        # use to fool mpc
        self.mpc_laks_active = cp_cam.vl["ACC_MPC_STATE"]["LKAS_Active"] != 0

        self.acc_hud_adas_counter = cp_cam.vl["ACC_HUD_ADAS"]["Counter"]
        self.acc_mpc_state_counter = cp_cam.vl["ACC_MPC_STATE"]["Counter"]
        self.acc_cmd_counter = cp_cam.vl["ACC_CMD"]["Counter"]

        self.cam_lkas = copy.copy(cp_cam.vl["ACC_MPC_STATE"])
        self.cam_adas = copy.copy(cp_cam.vl["ACC_HUD_ADAS"])
        self.cam_acc = copy.copy(cp_cam.vl["ACC_CMD"])
        self.esc_eps = copy.copy(cp.vl["ACC_EPS_STATE"])

        # safety checks to engage
        can_gear = int(cp.vl["DRIVE_STATE"]['GEAR'])

        ret.doorOpen = any([cp.vl["METER_CLUSTER"]['BACK_LEFT_DOOR'],
                            cp.vl["METER_CLUSTER"]['FRONT_LEFT_DOOR'],
                            cp.vl["METER_CLUSTER"]['BACK_RIGHT_DOOR'],
                            cp.vl["METER_CLUSTER"]['FRONT_RIGHT_DOOR']])

        ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]['SEATBELT_DRIVER'] == 0
        ret.gearShifter = self.parse_gear_shifter(
            self.shifter_values.get(can_gear, None))

        disengage = ret.doorOpen or ret.seatbeltUnlatched or ret.brakeHoldActive
        if disengage:
            self.is_cruise_latch = False

        # gas pedal
        ret.gas = cp.vl["PEDAL"]['GAS_PEDAL']
        ret.gasPressed = ret.gas > 0.01

        # brake pedal
        ret.brake = cp.vl["PEDAL"]['BRAKE_PEDAL']
        ret.brakePressed = bool(
            cp.vl["DRIVE_STATE"]["BRAKE_PRESSED"]) or ret.brake > 0.01

        # steer
        ret.steeringAngleDeg = cp.vl["STEER_MODULE_2"]['STEER_ANGLE_2']
        steer_dir = 1 if (ret.steeringAngleDeg - self.prev_angle >= 0) else -1
        self.prev_angle = ret.steeringAngleDeg
        ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE']
        ret.steeringTorqueEps = cp.vl["STEER_MODULE_2"]['DRIVER_EPS_TORQUE'] * steer_dir
        ret.steeringPressed = bool(abs(ret.steeringTorqueEps) > 6)
       # ret.steerWarning = False
        # ret.steerError = False       # TODO

        # TODO: get the real value
        ret.stockAeb = False
        ret.stockFcw = False
        ret.cruiseState.available = any(
            [cp.vl["ACC_HUD_ADAS"]["ACC_ON1"], cp.vl["ACC_HUD_ADAS"]["ACC_ON2"]])

        # distance_val = int(cp.vl["ACC_HUD_ADAS"]['SET_DISTANCE'])
        # ret.cruiseState.setDistance = self.parse_set_distance(
        #   self.set_distance_values.get(distance_val, None))

        # engage and disengage logic, do we still need this?
        if (cp.vl["PCM_BUTTONS"]["SET_BTN"] != 0 or cp.vl["PCM_BUTTONS"]["RES_BTN"] != 0) and not ret.brakePressed:
            self.is_cruise_latch = True

        # this can override the above engage disengage logic
        if bool(cp.vl["ACC_CMD"]["ACC_REQ_NOT_STANDSTILL"]):
            self.is_cruise_latch = True

        # byd speedCluster will follow wheelspeed if cruiseState is not available
        if ret.cruiseState.available:
            ret.cruiseState.speedCluster = max(
                int(cp.vl["ACC_HUD_ADAS"]['SET_SPEED']), 30) * CV.KPH_TO_MS
        else:
            ret.cruiseState.speedCluster = 0

        ret.cruiseState.speed = ret.cruiseState.speedCluster
        ret.cruiseState.standstill = bool(cp.vl["ACC_CMD"]["STANDSTILL_STATE"])
        ret.cruiseState.nonAdaptive = False

        stock_acc_on = bool(cp.vl["ACC_CMD"]["ACC_CONTROLLABLE_AND_ON"])
        if not ret.cruiseState.available or ret.brakePressed or not stock_acc_on:
            self.is_cruise_latch = False

        ret.cruiseState.enabled = self.is_cruise_latch

        # button presses
        ret.leftBlinker = bool(cp.vl["STALKS"]["LEFT_BLINKER"])
        ret.rightBlinker = bool(cp.vl["STALKS"]["RIGHT_BLINKER"])
        ret.genericToggle = bool(cp.vl["STALKS"]["GENERIC_TOGGLE"])
        ret.espDisabled = False

        # blindspot sensors
        if self.CP.enableBsm:
            # used for lane change so its okay for the chime to work on both side.
            ret.leftBlindspot = bool(cp.vl["BSM"]["LEFT_APPROACH"])
            ret.rightBlindspot = bool(cp.vl["BSM"]["RIGHT_APPROACH"])

        # Camera Controls
        # self.lss_state = cp_cam.vl["LKAS_HUD_ADAS"]["LSS_STATE"]
        # self.lss_alert = cp_cam.vl["LKAS_HUD_ADAS"]["SETTINGS"]

        # EPS give up all inputs until restart
        ret.steerFaultPermanent = (cp.vl["ACC_EPS_STATE"]["TorqueFailed"] == 1)

        return ret

    @staticmethod
    def get_can_parser(CP):
        messages = [
            # ("EPS", 100),
            # ("CARSPEED", 50),
            # ("PEDAL", 50),
            # ("ACC_EPS_STATE", 50),
            ("STALKS", 1),
            # ("BCM", 1),
            ("DRIVE_STATE", 10),
            ("ACC_HUD_ADAS", 1),
            ("WHEEL_SPEED", 80),
            ("PEDAL", 50),
            ("METER_CLUSTER", 20),
            ("STEER_MODULE_2", 80),
            ("STEERING_TORQUE", 80),
            ("BSM", 20),
            ("ACC_CMD", 20),
            ("PCM_BUTTONS", 20),
            ("LKAS_HUD_ADAS", 10),
        ]

        return {Bus.pt: CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)}

    @staticmethod
    def get_cam_can_parser(CP):
        messages = [
            ("ACC_HUD_ADAS", 50),
            ("ACC_CMD", 50),
            ("ACC_MPC_STATE", 50),
        ]

        return {Bus.cam: CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)}
