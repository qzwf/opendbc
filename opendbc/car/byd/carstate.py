from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, BUTTONS

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
    def __init__(self, CP):
        super().__init__(CP)
        self.button_states = {button.event_type: False for button in BUTTONS}
        self.counter_prev = 0

    def update(self, can_parsers) -> structs.CarState:
        cp = can_parsers[Bus.pt]
        ret = structs.CarState.new_message()

        # --- Steering ---
        ret.steeringAngleDeg = cp.vl["STEER_MODULE_2"]["STEER_ANGLE_2"] * 0.1
        ret.steeringTorque = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"] * 0.1
        ret.steeringPressed = abs(ret.steeringTorque) > 3.0

        # --- Pedals ---
        ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] > 1.0
        ret.brake = cp.vl["PEDAL"]["BRAKE_PEDAL"] * 0.01
        ret.brakePressed = bool(cp.vl["DRIVE_STATE"]["BRAKE_PRESSED"])

        # Catch active-low pedal pressed signal
        if not ret.brakePressed:
            ret.brakePressed = not bool(cp.vl["PEDAL_PRESSED"]["PEDAL_PRESSED_ACTIVE_LOW"])

        # --- Gear ---
        gear_map = {
            1: structs.CarState.GearShifter.park,
            2: structs.CarState.GearShifter.reverse,
            4: structs.CarState.GearShifter.drive,
        }
        ret.gearShifter = gear_map.get(int(cp.vl["DRIVE_STATE"]["GEAR"]),
                                       structs.CarState.GearShifter.unknown)

        # --- Wheel speeds ---
        fl = cp.vl["WHEEL_SPEED"]["WHEELSPEED_FL"] * 0.1
        fr = cp.vl["WHEEL_SPEED"]["WHEELSPEED_FR"] * 0.1
        rl = cp.vl["WHEEL_SPEED"]["WHEELSPEED_BL"] * 0.1
        rr = cp.vl["WHEEL_SPEED"]["WHEELSPEED_BR"] * 0.1
        ret.wheelSpeeds.fl = fl
        ret.wheelSpeeds.fr = fr
        ret.wheelSpeeds.rl = rl
        ret.wheelSpeeds.rr = rr
        ret.vEgoRaw = (fl + fr + rl + rr) / 4.0
        ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

        # --- Cruise / ACC ---
        acc_on = bool(cp.vl["ACC_HUD_ADAS"]["ACC_ON1"]) and bool(cp.vl["ACC_HUD_ADAS"]["ACC_ON2"])
        ret.cruiseState.enabled = acc_on
        ret.cruiseState.speed = cp.vl["ACC_HUD_ADAS"]["SET_SPEED"] * 0.5
        ret.cruiseState.available = acc_on

        # LKAS active — stock LKAS signal exposed for reference
        lkas_active = (not bool(cp.vl["LKAS_HUD_ADAS"]["STEER_ACTIVE_ACTIVE_LOW"]) or
                       bool(cp.vl["LKAS_HUD_ADAS"]["STEER_ACTIVE_1_1"]))
        ret.stockLkas = lkas_active

        # --- Safety ---
        ret.seatbeltUnlatched = not bool(cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"])
        ret.doorOpen = any([
            cp.vl["METER_CLUSTER"]["FRONT_LEFT_DOOR"],
            cp.vl["METER_CLUSTER"]["FRONT_RIGHT_DOOR"],
            cp.vl["METER_CLUSTER"]["BACK_LEFT_DOOR"],
            cp.vl["METER_CLUSTER"]["BACK_RIGHT_DOOR"],
        ])

        # --- Blinkers ---
        ret.leftBlinker = bool(cp.vl["STALKS"]["LEFT_BLINKER"])
        ret.rightBlinker = bool(cp.vl["STALKS"]["RIGHT_BLINKER"])

        # --- Button events ---
        button_events = []
        for button in BUTTONS:
            if button.can_addr not in ("PCM_BUTTONS", "STALKS"):
                continue
            msg_val = bool(cp.vl[button.can_addr][button.can_msg])
            if msg_val != self.button_states[button.event_type]:
                event = structs.CarState.ButtonEvent.new_message()
                event.type = button.event_type
                event.pressed = msg_val
                button_events.append(event)
                self.button_states[button.event_type] = msg_val
        ret.buttonEvents = button_events

        return ret

    @staticmethod
    def get_can_parsers(CP):
        pt_messages = [
            ("STEER_MODULE_2", 0),
            ("STEERING_TORQUE", 0),
            ("PEDAL", 0),
            ("PEDAL_PRESSED", 0),
            ("DRIVE_STATE", 0),
            ("WHEEL_SPEED", 0),
            ("ACC_HUD_ADAS", 0),
            ("LKAS_HUD_ADAS", 0),
            ("METER_CLUSTER", 0),
            ("PCM_BUTTONS", 0),
            ("STALKS", 0),
        ]
        return {
            Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
        }
