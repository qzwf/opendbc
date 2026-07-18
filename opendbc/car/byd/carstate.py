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
        cp = can_parsers[Bus.pt]       # bus 0: car-side chassis CAN
        cp_cam = can_parsers[Bus.cam]  # bus 2: camera-side chassis CAN
        ret = structs.CarState.new_message()

        # --- Steering ---
        # DBC already applies 0.1 factor; cp.vl returns degrees / Nm directly
        ret.steeringAngleDeg = cp.vl["STEER_MODULE_2"]["STEER_ANGLE_2"]
        ret.steeringTorque = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"]
        ret.steeringPressed = abs(ret.steeringTorque) > 30.0

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
        # DBC factor 0.1 gives km/h, but BYD ATTO3 India raw values read ~32.5% high
        # vs odometer at steady-state. Correction: 40/53. Verify with GPS if re-calibrating.
        _SPD_CORR = 40.0 / 53.0
        fl = cp.vl["WHEEL_SPEED"]["WHEELSPEED_FL"] * _SPD_CORR / 3.6
        fr = cp.vl["WHEEL_SPEED"]["WHEELSPEED_FR"] * _SPD_CORR / 3.6
        rl = cp.vl["WHEEL_SPEED"]["WHEELSPEED_BL"] * _SPD_CORR / 3.6
        # WHEELSPEED_BR (bits 48-63): byte 7 is a constant status byte (0x41),
        # NOT the high byte of the wheel speed. DBC wrongly declares it as 16-bit.
        # Until the DBC is corrected, derive RR from the other three wheels.
        rr = (fl + fr + rl) / 3.0
        ret.wheelSpeeds.fl = fl
        ret.wheelSpeeds.fr = fr
        ret.wheelSpeeds.rl = rl
        ret.wheelSpeeds.rr = rr
        ret.vEgoRaw = (fl + fr + rl) / 3.0
        ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

        # --- Cruise / ACC --- (read from camera bus 2 — native source)
        acc_on = bool(cp_cam.vl["ACC_HUD_ADAS"]["ACC_ON1"]) and bool(cp_cam.vl["ACC_HUD_ADAS"]["ACC_ON2"])
        ret.cruiseState.enabled = acc_on
        # DBC SET_SPEED factor 0.5 already applied (gives km/h); convert to m/s
        ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SET_SPEED"] / 3.6
        ret.cruiseState.available = acc_on

        # Camera LKAS is always active on bus 2; don't surface it as stockLkas
        # because it would fire noEntry permanently and block all engagement
        ret.stockLkas = False

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
        # Bus 0: messages sent by car ECUs (EPS, BCM, wheel speed, pedals, etc.)
        pt_messages = [
            ("STEER_MODULE_2", 0),
            ("STEERING_TORQUE", 0),
            ("PEDAL", 0),
            ("PEDAL_PRESSED", 0),
            ("DRIVE_STATE", 0),
            ("WHEEL_SPEED", 0),
            ("METER_CLUSTER", 0),
            ("PCM_BUTTONS", 0),
            ("STALKS", 0),
        ]
        # Bus 2: messages sent by the ADAS camera module
        cam_messages = [
            ("ACC_HUD_ADAS", 0),
            ("LKAS_HUD_ADAS", 0),
        ]
        return {
            Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
            Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.cam], cam_messages, 2),
        }
