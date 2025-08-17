
from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import CanBus, BUTTONS

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
    def __init__(self, CP):
        super().__init__(CP)

        # Initialize vehicle state variables
        self.button_events = []
        self.button_states = {button.event_type: False for button in BUTTONS}

        # Steering state
        self.steering_angle = 0.
        self.steering_rate = 0.
        self.steering_torque = 0.
        self.steering_pressed = False
        self.steer_error = False

        # Pedal states
        self.pedal_gas = 0.
        self.pedal_brake = 0.
        self.brake_pressed = False

        # Vehicle motion
        self.v_ego_raw = 0.
        self.wheel_speeds = [0., 0., 0., 0.]

        # Gear and drive state
        self.gear_shifter = structs.CarParams.GearShifter.park
        self.gear_shifter_prev = structs.CarParams.GearShifter.park

        # ADAS states
        self.acc_active = False
        self.main_on = False
        self.cruise_speed = 0.
        self.lkas_enabled = False

        # Safety states
        self.door_open = False
        self.seatbelt_driver = True

        # EV-specific states
        self.charging_state = False
        self.battery_percent = 50.  # Default to 50% if not available

        # Previous values for change detection
        self.gear_shifter_prev = self.gear_shifter

    def update(self, cp, cp_cam):
        ret = structs.CarState()

        # Update steering state from STEER_MODULE_2 and STEERING_MODULE_ADAS
        if cp.vl_all["STEER_MODULE_2"]["COUNTER"] != self.counter_prev:
            self.steering_angle = cp.vl["STEER_MODULE_2"]["STEER_ANGLE_2"] * 0.1  # Convert to degrees
            self.steering_torque = cp.vl["STEER_MODULE_2"]["DRIVER_EPS_TORQUE"]

        # ADAS steering angle from camera
        if "STEERING_MODULE_ADAS" in cp.vl_all and cp.vl_all["STEERING_MODULE_ADAS"]["COUNTER"] != self.adas_counter_prev:
            adas_angle = cp.vl["STEERING_MODULE_ADAS"]["STEER_ANGLE"] * 0.1
            # Use ADAS angle when available, otherwise use EPS angle
            if abs(adas_angle) > 0.1:
                self.steering_angle = adas_angle

        # Steering torque and status
        if "STEERING_TORQUE" in cp.vl_all:
            _main_torque = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"] * 0.1
            self.steering_pressed = abs(self.steering_torque) > 3.0

        # Pedal states from PEDAL message
        if "PEDAL" in cp.vl_all:
            self.pedal_gas = cp.vl["PEDAL"]["GAS_PEDAL"] * 0.01
            self.pedal_brake = cp.vl["PEDAL"]["BRAKE_PEDAL"] * 0.01

        # Brake pressed state from DRIVE_STATE
        if "DRIVE_STATE" in cp.vl_all:
            self.brake_pressed = bool(cp.vl["DRIVE_STATE"]["BRAKE_PRESSED"])

            # Gear state mapping
            gear_map = {1: structs.CarParams.GearShifter.park,
                       2: structs.CarParams.GearShifter.reverse,
                       4: structs.CarParams.GearShifter.drive}
            self.gear_shifter = gear_map.get(cp.vl["DRIVE_STATE"]["GEAR"], structs.CarParams.GearShifter.unknown)

        # Additional brake detection from PEDAL_PRESSED
        if "PEDAL_PRESSED" in cp.vl_all:
            # Active low signal
            pedal_pressed = not bool(cp.vl["PEDAL_PRESSED"]["PEDAL_PRESSED_ACTIVE_LOW"])
            self.brake_pressed = self.brake_pressed or pedal_pressed

        # Wheel speeds from WHEEL_SPEED
        if "WHEEL_SPEED" in cp.vl_all:
            self.wheel_speeds = [
                cp.vl["WHEEL_SPEED"]["WHEELSPEED_FL"] * 0.1,
                cp.vl["WHEEL_SPEED"]["WHEELSPEED_FR"] * 0.1,
                cp.vl["WHEEL_SPEED"]["WHEELSPEED_BL"] * 0.1,
                cp.vl["WHEEL_SPEED"]["WHEELSPEED_BR"] * 0.1,
            ]
            self.v_ego_raw = sum(self.wheel_speeds) / 4.0

        # ACC and cruise control state from ACC_HUD_ADAS
        if "ACC_HUD_ADAS" in cp.vl_all:
            self.acc_active = bool(cp.vl["ACC_HUD_ADAS"]["ACC_ON1"]) and bool(cp.vl["ACC_HUD_ADAS"]["ACC_ON2"])
            self.cruise_speed = cp.vl["ACC_HUD_ADAS"]["SET_SPEED"] * 0.5  # Convert to km/h

        # LKAS state from LKAS_HUD_ADAS
        if "LKAS_HUD_ADAS" in cp.vl_all:
            # LKAS active when steering active signals are set (inverted logic)
            lkas_active_1 = not bool(cp.vl["LKAS_HUD_ADAS"]["STEER_ACTIVE_ACTIVE_LOW"])
            lkas_active_2 = bool(cp.vl["LKAS_HUD_ADAS"]["STEER_ACTIVE_1_1"])
            lkas_active_3 = bool(cp.vl["LKAS_HUD_ADAS"]["STEER_ACTIVE_1_2"])
            lkas_active_4 = bool(cp.vl["LKAS_HUD_ADAS"]["STEER_ACTIVE_1_3"])

            self.lkas_enabled = lkas_active_1 or lkas_active_2 or lkas_active_3 or lkas_active_4
            self.main_on = self.lkas_enabled or self.acc_active

        # Door and safety states from METER_CLUSTER
        if "METER_CLUSTER" in cp.vl_all:
            self.seatbelt_driver = bool(cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"])

            # Door states
            doors = [
                cp.vl["METER_CLUSTER"]["FRONT_LEFT_DOOR"],
                cp.vl["METER_CLUSTER"]["FRONT_RIGHT_DOOR"],
                cp.vl["METER_CLUSTER"]["BACK_LEFT_DOOR"],
                cp.vl["METER_CLUSTER"]["BACK_RIGHT_DOOR"],
            ]
            self.door_open = any(doors)

        # Button events from PCM_BUTTONS and STALKS
        self.button_events = []
        if "PCM_BUTTONS" in cp.vl_all:
            for button in BUTTONS:
                if button.can_addr == "PCM_BUTTONS":
                    button_pressed = bool(cp.vl["PCM_BUTTONS"][button.can_msg])
                    if button_pressed != self.button_states[button.event_type]:
                        event = structs.CarState.ButtonEvent.new_message()
                        event.type = button.event_type
                        event.pressed = button_pressed
                        self.button_events.append(event)
                        self.button_states[button.event_type] = button_pressed

        if "STALKS" in cp.vl_all:
            for button in BUTTONS:
                if button.can_addr == "STALKS":
                    button_pressed = bool(cp.vl["STALKS"][button.can_msg])
                    if button_pressed != self.button_states[button.event_type]:
                        event = structs.CarState.ButtonEvent.new_message()
                        event.type = button.event_type
                        event.pressed = button_pressed
                        self.button_events.append(event)
                        self.button_states[button.event_type] = button_pressed

        # Store previous values for change detection
        self.counter_prev = cp.vl_all["STEER_MODULE_2"]["COUNTER"] if "STEER_MODULE_2" in cp.vl_all else 0
        self.adas_counter_prev = cp.vl_all["STEERING_MODULE_ADAS"]["COUNTER"] if "STEERING_MODULE_ADAS" in cp.vl_all else 0
        self.gear_shifter_prev = self.gear_shifter

        return ret

    @staticmethod
    def get_can_parser(CP):
        # CAN message definitions for parsing
        messages = [
            # Steering and vehicle control
            ("STEER_MODULE_2", 50),           # 287 - Secondary steering module
            ("STEERING_MODULE_ADAS", 50),     # 482 - ADAS steering control
            ("STEERING_TORQUE", 50),          # 508 - Main steering torque

            # Pedals and braking
            ("PEDAL", 50),                    # 834 - Gas and brake pedals
            ("PEDAL_PRESSED", 10),            # 544 - Pedal press detection
            ("DRIVE_STATE", 10),              # 578 - Gear and brake state

            # Vehicle motion
            ("WHEEL_SPEED", 50),              # 290 - Individual wheel speeds
            ("WHEELSPEED_CLEAN", 50),         # 496 - Processed wheel speed

            # ADAS and cruise control
            ("ACC_HUD_ADAS", 10),             # 813 - ACC status and HUD
            ("LKAS_HUD_ADAS", 10),            # 790 - LKAS status and HUD
            ("ACC_CMD", 50),                  # 814 - ACC commands (read-only)

            # User controls
            ("PCM_BUTTONS", 10),              # 944 - Steering wheel buttons
            ("STALKS", 10),                   # 307 - Turn signal stalks

            # Safety and status
            ("METER_CLUSTER", 10),            # 660 - Dashboard indicators
            ("BSM", 10),                      # 1048 - Blind spot monitoring
        ]

        return CANParser(messages, CanBus.pt)

    @staticmethod
    def get_cam_can_parser(CP):
        # Camera CAN messages (if any)
        messages = []
        return CANParser(messages, CanBus.cam)
