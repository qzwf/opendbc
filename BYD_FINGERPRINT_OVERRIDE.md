# BYD ATTO3 Fingerprint Override Guide

## Overview
This guide explains how openpilot fingerprints vehicles and provides methods to force BYD ATTO3 recognition when automatic detection fails.

---

## How Fingerprinting Works

OpenPilot uses a **3-stage fingerprinting system** (defined in `opendbc/car/car_helpers.py`):

### Stage 1: CAN Bus Fingerprinting (Primary)
**Location**: `car_helpers.py:42` - `can_fingerprint()` function

**Process**:
1. Monitors CAN bus for 1 second (100 frames @ 100Hz)
2. Records message IDs and data lengths from bus 0 and bus 1
3. Compares against known patterns in `byd/fingerprints.py`
4. Eliminates incompatible vehicles until one candidate remains

**Your BYD ATTO3 Fingerprint**: 154 unique CAN messages detected
- Key messages: `482` (STEERING_MODULE_ADAS), `790` (LKAS_HUD), `813` (ACC_HUD), `814` (ACC_CMD)
- Currently stored in `byd/fingerprints.py` with 5 different fingerprint variants

### Stage 2: Firmware Version Matching (Secondary)
**Location**: `car_helpers.py:84` - `fingerprint()` function

**Process**:
1. Queries ECU firmware via UDS diagnostic protocol (OBD-II)
2. Compares firmware strings against `FW_VERSIONS` dict in `byd/fingerprints.py`
3. If exactly 1 candidate matches, uses firmware-based detection

**Your BYD ATTO3 ECUs Detected**:
- Engine ECU (0x7E0): `b'H7\\x00\\x11V\\xfd\\x00\\x12!'`
- HVAC ECU (0x7B3): `b'\\xf1\\x8b\\x00\\x00\\x00\\xff'`
- Motor Controller 1 (0x321): `b'\\xd6\\xdd\\x00\\x8f\\x00\\x00\\x01\\xbc'`
- Motor Controller 2 (0x322): Multiple firmware versions detected
- Charging System (0x323): `b'\\xff\\xff\\x3f\\x9c\\xff\\xfc\\x09\\x22'`

### Stage 3: Priority Override System
**Precedence Order** (highest to lowest):
1. **`FINGERPRINT` environment variable** ← **THIS IS WHAT YOU WANT**
2. Firmware version match (if exactly 1 candidate)
3. CAN fingerprint match
4. Falls back to "MOCK" (simulation mode)

---

## Why Automatic Fingerprinting Might Fail

1. **Missing CAN messages**: If your car doesn't transmit all expected messages
2. **Different firmware versions**: Your ECU firmware doesn't match known versions
3. **Bus configuration issues**: Messages on wrong CAN bus
4. **Incomplete fingerprints**: Database needs more fingerprint variants
5. **Timing issues**: Fingerprint timeout before enough data collected

---

## Solution: Force BYD ATTO3 Recognition

### ✅ **Method 1: Environment Variable (RECOMMENDED)**

This is the **cleanest and safest** method. No code changes required.

#### On Your Comma 3X Device:

1. **SSH into the device**:
   ```bash
   ssh comma@<device-ip>  # Password is usually shown on device screen
   ```

2. **Set the environment variable**:
   ```bash
   export FINGERPRINT="BYD ATTO3"
   ```

3. **Make it permanent** - Add to launch script:
   ```bash
   # Edit the openpilot launch configuration
   echo 'export FINGERPRINT="BYD ATTO3"' >> /data/bashrc
   ```

4. **Restart the device**:
   ```bash
   reboot
   ```

#### On Your Development Machine (for testing):

```bash
# In your terminal before running openpilot
export FINGERPRINT="BYD ATTO3"

# Then run your tests
cd /home/qzwf/openpilot/opendbc_repo
python3 -m pytest opendbc/car/byd/tests/
```

#### Or use the convenience script:

```bash
source /home/qzwf/openpilot/opendbc_repo/force_byd_atto3.sh
```

---

### 🔧 **Method 2: Code Modification (PERMANENT)**

I've already modified `car_helpers.py` to add a fallback that forces BYD ATTO3 when no fingerprint matches.

**What was changed** (line 143-149 in `car_helpers.py`):
```python
# Force BYD ATTO3 if no fingerprint matched (bypass for development/testing)
# REMOVE THIS IN PRODUCTION - for BYD ATTO3 development only
if car_fingerprint is None:
  carlog.warning("No fingerprint match - forcing BYD ATTO3 for development")
  car_fingerprint = "BYD ATTO3"
  source = CarParams.FingerprintSource.fixed
```

**⚠️ Important**: 
- This change makes BYD ATTO3 the default for ANY unrecognized vehicle
- Only use this for development on your personal vehicle
- **DO NOT** submit this to upstream openpilot
- Revert this before testing with other vehicles

---

## How to Verify It's Working

### 1. Check Environment Variable:
```bash
echo $FINGERPRINT
# Should output: BYD ATTO3
```

### 2. Test Fingerprinting:
```bash
cd /home/qzwf/openpilot/opendbc_repo
python3 << EOF
import os
print("FINGERPRINT env var:", os.environ.get('FINGERPRINT', 'NOT SET'))

from opendbc.car.byd.values import CAR
print("BYD ATTO3 value:", CAR.BYD_ATTO3)
EOF
```

### 3. Monitor Openpilot Logs:
On the Comma 3X, check logs after startup:
```bash
tail -f /data/community/crashes/*.log | grep -i fingerprint
```

You should see:
```
{"event": "fingerprinted", "car_fingerprint": "BYD ATTO3", "source": "fixed", ...}
```

---

## Next Steps After Forcing Fingerprint

Once BYD ATTO3 is recognized, the system will:

1. ✅ Load `byd/interface.py` - Car interface implementation
2. ✅ Load `byd/carstate.py` - Parse CAN messages from your car  
3. ✅ Load `byd/carcontroller.py` - Send control commands to your car
4. ✅ Use DBC file `byd_general.dbc` for CAN message definitions
5. ✅ Apply tuning parameters from `byd/values.py`

### What Still Needs Work:

The current implementation is **minimal** - it will recognize your car but won't control it yet. You need to:

1. **Implement `CarState.update()`** - Parse steering angle, speed, pedals, buttons from CAN
2. **Implement `CarController.update()`** - Send LKAS steering commands to CAN
3. **Test and tune** - Adjust steering PID/torque parameters
4. **Safety validation** - Ensure proper driver monitoring and failsafes

---

## Troubleshooting

### Issue: Still shows "Unrecognized Car"
**Solution**: 
- Verify environment variable is set: `echo $FINGERPRINT`
- Check you've restarted openpilot after setting the variable
- Look for typos - must be exact: `"BYD ATTO3"` (with space, not underscore)

### Issue: Openpilot crashes after forcing fingerprint
**Solution**:
- Check `byd/interface.py` has all required methods implemented
- Verify DBC file `byd_general.dbc` exists and is valid
- Review logs: `tail -f /data/community/crashes/*.log`

### Issue: Want to switch back to auto-detect
**Solution**:
```bash
unset FINGERPRINT
# Or remove from /data/bashrc and reboot
```

### Issue: Environment variable not persisting after reboot
**Solution**:
- Add to `/data/bashrc` (Comma 3X) or `~/.bashrc` (dev machine)
- Make sure the file is sourced on startup
- For Comma 3X, may need to add to openpilot's systemd service file

---

## File Reference

| File | Purpose |
|------|---------|
| `opendbc/car/car_helpers.py:86` | Reads `FINGERPRINT` env var |
| `opendbc/car/car_helpers.py:140-142` | Applies fixed fingerprint override |
| `opendbc/car/byd/fingerprints.py` | CAN and FW fingerprint database |
| `opendbc/car/byd/values.py` | Car parameters and ECU configuration |
| `opendbc/car/byd/interface.py` | Main car interface - needs implementation |
| `opendbc/car/values.py:3,20` | Registers BYD brand with openpilot |

---

## Additional Environment Variables

You can combine these for different behaviors:

```bash
# Force BYD ATTO3 recognition
export FINGERPRINT="BYD ATTO3"

# Skip firmware query (faster startup, but less verification)
export SKIP_FW_QUERY=1

# Disable firmware caching (force fresh detection each time)
export DISABLE_FW_CACHE=1
```

---

## Summary

**Recommended approach for your BYD ATTO3**:

1. ✅ Use environment variable: `export FINGERPRINT="BYD ATTO3"`
2. ✅ Make it permanent by adding to `/data/bashrc` on Comma 3X
3. ✅ Keep the code modification in `car_helpers.py` as backup
4. ✅ Monitor logs to verify it's working
5. ✅ Continue developing CarState and CarController implementations

This will bypass fingerprinting entirely and let you proceed with testing and development of the actual control logic.

---

## Support

If you encounter issues:
1. Check environment variable is set correctly
2. Review openpilot logs for error messages
3. Verify all BYD files are present and syntax-valid
4. Test with the convenience script: `source force_byd_atto3.sh`
5. Reference your external context notes for confirmed ECU addresses and CAN messages

Good luck with your BYD ATTO3 port! 🚗💨
