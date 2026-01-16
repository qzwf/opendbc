# BYD ATTO3 OpenPilot Integration

## Overview
This directory contains the complete implementation for BYD ATTO3 electric vehicle integration with OpenPilot/opendbc. The implementation is based on comprehensive CAN fingerprinting, DBC analysis, and firmware version extraction from actual BYD ATTO3 vehicles.

## Files Structure

### Core Implementation Files
- **`values.py`** - Vehicle specifications, ECU definitions, button mappings, and FW query configuration
- **`fingerprints.py`** - CAN fingerprints and firmware versions for vehicle detection
- **`interface.py`** - Main car interface class with parameter configuration
- **`carstate.py`** - Vehicle state monitoring and CAN message parsing
- **`carcontroller.py`** - Vehicle control commands (steering, ACC, HUD)
- **`bydcan.py`** - CAN message creation and encoding functions
- **`__init__.py`** - Module initialization

### Supporting Files  
- **`tests/test_byd.py`** - Unit tests for BYD implementation
- **`README.md`** - This documentation file

## Technical Specifications

### Vehicle Information
- **Model**: BYD ATTO3 Electric Vehicle (2022-2024)
- **Mass**: 1750 kg
- **Wheelbase**: 2.72 m  
- **Steering Ratio**: 14.8:1
- **Type**: Battery Electric Vehicle (BEV)

### CAN Bus Configuration
- **Bus 0 (pt)**: Primary powertrain and control messages
- **Bus 1 (cam)**: Camera and vision system messages  
- **Bus 2 (radar)**: Radar messages (if equipped)
- **CAN-FD**: Supported for extended message lengths

### Fingerprint Data
The implementation includes comprehensive fingerprinting data:
- **154 unique CAN messages** detected from actual vehicle
- **19/20 DBC messages** confirmed in live testing
- **Multiple fingerprint variants** for different configurations

### Firmware Versions
Extracted firmware versions from actual ECUs:
- **Engine ECU (0x7E0)**: `b'"\\xaa\\xaa\\xaa\\xaa'`
- **HVAC ECU (0x7B3)**: `b'\\xf1\\x8b\\x00\\x00\\x00\\xff'`

## Key Features Supported

### ADAS Features
- **Lane Keep Assist (LKAS)** - Via STEERING_MODULE_ADAS (482) and LKAS_HUD_ADAS (790)
- **Adaptive Cruise Control (ACC)** - Via ACC_CMD (814) and ACC_HUD_ADAS (813)  
- **Blind Spot Monitoring (BSM)** - Via BSM message (1048)
- **Traffic Sign Recognition** - Partial support via LKAS_HUD_ADAS

### Vehicle Controls
- **Steering Control** - Torque-based via STEERING_MODULE_ADAS
- **Longitudinal Control** - ACC command integration (experimental)
- **HUD Integration** - Both LKAS and ACC status displays

### Safety Features
- **Driver Monitoring** - Steering torque and engagement detection
- **Door Status** - All door positions from METER_CLUSTER
- **Seatbelt Detection** - Driver seatbelt status
- **Gear Detection** - Park, Reverse, Drive states

### EV-Specific Features
- **Battery Management** - ECU detection for 0x320
- **Motor Control** - Dual motor controller support (0x321, 0x322)
- **Charging System** - Charging state monitoring (0x323)

## CAN Message Mapping

### Critical ADAS Messages
| ID | Name | Function | Frequency |
|----|------|----------|-----------|
| 482 | STEERING_MODULE_ADAS | LKAS steering control | 50 Hz |
| 508 | STEERING_TORQUE | Main steering torque | 50 Hz |
| 790 | LKAS_HUD_ADAS | LKAS status display | 10 Hz |
| 813 | ACC_HUD_ADAS | ACC status display | 10 Hz |
| 814 | ACC_CMD | ACC acceleration commands | 50 Hz |

### Vehicle State Messages  
| ID | Name | Function | Frequency |
|----|------|----------|-----------|
| 287 | STEER_MODULE_2 | Secondary steering data | 50 Hz |
| 290 | WHEEL_SPEED | Individual wheel speeds | 50 Hz |
| 307 | STALKS | Turn signals and controls | 10 Hz |
| 578 | DRIVE_STATE | Gear and brake state | 10 Hz |
| 660 | METER_CLUSTER | Dashboard indicators | 10 Hz |
| 834 | PEDAL | Gas and brake pedals | 50 Hz |
| 944 | PCM_BUTTONS | Steering wheel buttons | 10 Hz |

## Control Parameters

### Steering Control
```python
STEER_MAX = 300                    # Maximum steering torque
STEER_DELTA_UP = 17               # Torque increase rate
STEER_DELTA_DOWN = 17             # Torque decrease rate
STEER_DRIVER_ALLOWANCE = 68       # Driver intervention threshold
STEER_DRIVER_MULTIPLIER = 3       # Driver torque scaling
```

### Lateral Tuning (PID Controller)
```python
kpV = [0.25]    # Proportional gain
kiV = [0.05]    # Integral gain  
kf = 0.00004    # Feedforward gain
```

### Longitudinal Tuning (Experimental)
```python
kpBP = [0., 5., 35.]              # Speed breakpoints
kpV = [1.2, 0.8, 0.5]            # Proportional gains
kiBP = [0., 35.]                  # Integral breakpoints  
kiV = [0.18, 0.12]               # Integral gains
```

## Installation and Usage

### Prerequisites
1. **Hardware**: Comma3x device with BYD-compatible harness
2. **Software**: OpenPilot 0.9.8+ with opendbc integration
3. **Vehicle**: BYD ATTO3 2022-2024 model year

### Installation Steps
1. Copy BYD folder to `opendbc/car/byd/`
2. Add BYD DBC file to `opendbc/dbc/byd_general.dbc`
3. Update main opendbc imports to include BYD
4. Flash to Comma3x device
5. Perform initial fingerprinting drive

### Testing
Run unit tests to verify implementation:
```bash
python -m pytest opendbc/car/byd/tests/test_byd.py
```

## Safety Considerations

### Driver Responsibility
- **Always supervise** the vehicle when openpilot is engaged
- **Keep hands near steering wheel** and be ready to take control
- **Monitor vehicle behavior** especially during initial testing

### System Limitations
- **Experimental longitudinal control** - Use with caution
- **Weather limitations** - Reduced performance in poor conditions  
- **Construction zones** - May require manual intervention
- **Local regulations** - Ensure compliance with local laws

### Fail-Safe Mechanisms
- **Driver torque override** - Immediate disengagement on steering input
- **Speed limitations** - Reduced performance at very low/high speeds
- **Error detection** - System disengages on CAN errors
- **Watchdog timers** - Automatic disengagement if communication lost

## Development and Debugging

### Adding New Features
1. Analyze DBC file for relevant CAN messages
2. Add message parsing to `carstate.py`
3. Add control logic to `carcontroller.py`  
4. Update CAN message creation in `bydcan.py`
5. Test thoroughly before deployment

### Firmware Version Updates
1. Use firmware extraction tools to get new versions
2. Add to `FW_VERSIONS` in `fingerprints.py`
3. Test vehicle detection with new firmware
4. Submit updates to community

### Troubleshooting
- **Vehicle not detected**: Check fingerprint matches
- **Controls not working**: Verify CAN message formats
- **Intermittent issues**: Check CAN bus timing and counters
- **Safety errors**: Review steering limits and driver detection

## Community and Support

### Contributing
- Submit pull requests with thorough testing
- Include vehicle year/model information
- Follow existing code style and conventions
- Add appropriate unit tests

### Resources
- **OpenPilot Docs**: https://docs.comma.ai/
- **BYD ATTO3 Specs**: Vehicle technical documentation
- **CAN Database**: DBC file with message definitions
- **Community Forums**: Discussion and support

## Legal and Warranty

### Disclaimer
This software is provided "as is" without warranty. Use at your own risk. Vehicle modifications may void warranty and may not be legal in all jurisdictions.

### Compliance
Ensure compliance with local regulations regarding:
- Vehicle modifications
- Autonomous driving systems
- Safety equipment requirements
- Insurance coverage

---

**Last Updated**: December 2024  
**Implementation Version**: 1.0  
**Tested Vehicles**: BYD ATTO3 2022-2024  
**OpenPilot Compatibility**: 0.9.8+
