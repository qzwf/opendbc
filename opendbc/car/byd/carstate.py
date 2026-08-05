from opendbc.can.parser import CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, BUTTONS

ButtonType = structs.CarState.ButtonEvent.Type

# LKAS_HUD_ADAS fields the camera owns; we mirror them into our own copy of the message
LKAS_HUD_PASSTHROUGH = ("LSS_STATE", "SETTINGS", "SET_ME_XFF", "SET_ME_X5F",
                        "TSR", "HMA", "PT2", "PT3", "PT4", "PT5")

# Bytes 0-1 and SET_ME_XE of STEERING_MODULE_ADAS are not three independent fields: they
# form one 20-bit sequence, (SET_ME_XE << 16) | (byte1 << 8) | byte0, which the camera
# advances by a fixed increment on every frame of a steering episode (measured: +1025/frame
# at 115 km/h, +25575/frame in city driving). It is zero while the camera is idle. We can
# neither derive the value nor the increment, and the EPS ignores commands where they look
# wrong, so we latch the camera's sequence and keep advancing it ourselves while we steer.
STEER_SEQ_BITS = 20
STEER_SEQ_MASK = (1 << STEER_SEQ_BITS) - 1

# ACC_HUD_ADAS arrives at 50Hz with no counter/checksum validation in the parser, and a
# single corrupted frame used to drop cruiseState.enabled for one frame — enough to fire
# pcmDisable and drop panda's controls_allowed. Require a few consecutive frames to drop.
CRUISE_DROP_FRAMES = 5


def pack_steer_seq(unknown: int, set_me_x01: int, set_me_xe: int) -> int:
    """Fold the three decoded DBC fields back into the single 20-bit sequence value."""
    byte0 = (unknown >> 6) & 0xFF
    byte1 = ((unknown & 0x3F) << 2) | (set_me_x01 & 0x3)
    return ((set_me_xe & 0xF) << 16) | (byte1 << 8) | byte0


def unpack_steer_seq(seq: int) -> dict:
    """Split the 20-bit sequence back into the DBC field values the packer expects."""
    byte0, byte1 = seq & 0xFF, (seq >> 8) & 0xFF
    return {"UNKNOWN": (byte0 << 6) | (byte1 >> 2),
            "SET_ME_X01": byte1 & 0x3,
            "SET_ME_XE": (seq >> 16) & 0xF}


class CarState(CarStateBase):
    def __init__(self, CP):
        super().__init__(CP)
        self.button_states = {button.event_type: False for button in BUTTONS}
        self.lkas_hud = dict.fromkeys(LKAS_HUD_PASSTHROUGH, 0)
        self.steer_seq = 0          # camera's 20-bit steering sequence value
        self.steer_seq_step = 0     # its per-frame increment
        self._seq_prev = None
        self.camera_has_steered = False
        self.acc_off_frames = 0
        self.cruise_enabled_last = False
        self.prev_angle = 0.0

    def update(self, can_parsers) -> structs.CarState:
        cp = can_parsers[Bus.pt]       # bus 0: car-side chassis CAN
        cp_cam = can_parsers[Bus.cam]  # bus 2: camera-side chassis CAN
        ret = structs.CarState.new_message()

        # --- Steering ---
        ret.steeringAngleDeg = cp.vl["STEER_MODULE_2"]["STEER_ANGLE_2"]
        ret.steeringRateDeg = (ret.steeringAngleDeg - self.prev_angle) / DT_CTRL
        self.prev_angle = ret.steeringAngleDeg
        # DRIVER_EPS_TORQUE (byte 2 of STEER_MODULE_2): actual column torque sensor, raw 0–255 unsigned.
        # MAIN_TORQUE (STEERING_TORQUE 0x1FC) is total EPS motor output — NOT driver input.
        ret.steeringTorque = cp.vl["STEER_MODULE_2"]["DRIVER_EPS_TORQUE"]
        ret.steeringTorqueEps = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"]
        ret.steeringPressed = ret.steeringTorque > 80  # raw threshold; observed max ~52 during normal turns

        # --- Pedals ---
        ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] > 1.0
        brake_pedal = cp.vl["PEDAL"]["BRAKE_PEDAL"]  # physical, DBC factor 0.01 already applied
        ret.brake = min(brake_pedal, 1.0)
        # BRAKE_PEDAL is the only signal that tracks the driver's foot. Verified against a
        # controlled pedal capture and against 3.6h of driving:
        #   - DRIVE_STATE.BRAKE_PRESSED (DBC bit 37) is dead — byte 4 is a constant 0x0C.
        #   - PEDAL_PRESSED_ACTIVE_LOW is the brake-light switch: 86% of its assertions on
        #     the road happened while the camera's ACC was commanding decel, not the driver.
        ret.brakePressed = brake_pedal > 0.03  # raw > 3, matches BYD_BRAKE_THRESHOLD in panda

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
        ret.standstill = ret.vEgoRaw < 0.05

        # --- Cruise / ACC --- (read from camera bus 2 — native source)
        acc_on1 = bool(cp_cam.vl["ACC_HUD_ADAS"]["ACC_ON1"])
        acc_on2 = bool(cp_cam.vl["ACC_HUD_ADAS"]["ACC_ON2"])

        # Debounce the drop: a single corrupted frame must not disengage.
        if acc_on1 and acc_on2:
            self.acc_off_frames = 0
        else:
            self.acc_off_frames += 1
        if self.acc_off_frames == 0:
            self.cruise_enabled_last = True
        elif self.acc_off_frames >= CRUISE_DROP_FRAMES:
            self.cruise_enabled_last = False

        ret.cruiseState.enabled = self.cruise_enabled_last
        # available is the ACC main switch, not the engaged state — aliasing the two made
        # every disengage also raise wrongCarMode and block re-engagement.
        ret.cruiseState.available = acc_on1 or acc_on2
        # DBC SET_SPEED factor 0.5 already applied (gives km/h); convert to m/s
        ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SET_SPEED"] / 3.6
        ret.cruiseState.standstill = False

        # We take over LKAS entirely: panda blocks the camera's steering command and we
        # send our own, so the stock system is never a competing controller.
        ret.stockLkas = False
        self.lkas_hud = {k: cp_cam.vl["LKAS_HUD_ADAS"][k] for k in LKAS_HUD_PASSTHROUGH}

        # Track the camera's steering sequence while it is the one driving the EPS
        cam_steer = cp_cam.vl["STEERING_MODULE_ADAS"]
        if cam_steer["STEER_REQ"]:
            seq = pack_steer_seq(int(cam_steer["UNKNOWN"]), int(cam_steer["SET_ME_X01"]),
                                 int(cam_steer["SET_ME_XE"]))
            if seq:
                if self._seq_prev is not None and seq != self._seq_prev:
                    self.steer_seq_step = (seq - self._seq_prev) & STEER_SEQ_MASK
                self._seq_prev = seq
                self.steer_seq = seq
                self.camera_has_steered = True
        else:
            self._seq_prev = None

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
            ("STEERING_MODULE_ADAS", 0),
        ]
        return {
            Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
            Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.cam], cam_messages, 2),
        }
