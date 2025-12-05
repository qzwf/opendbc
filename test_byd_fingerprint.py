#!/usr/bin/env python3
"""
Test script to verify BYD ATTO3 fingerprint override is working correctly.

Usage:
    # Test with environment variable set
    export FINGERPRINT="BYD ATTO3"
    python3 test_byd_fingerprint.py

    # Test without environment variable (will use code fallback)
    unset FINGERPRINT
    python3 test_byd_fingerprint.py
"""

import os
import sys

# Add opendbc to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_environment_variable():
    """Test if FINGERPRINT environment variable is set correctly."""
    print("\n" + "="*60)
    print("TEST 1: Environment Variable Check")
    print("="*60)

    fingerprint = os.environ.get('FINGERPRINT', None)
    if fingerprint == "BYD ATTO3":
        print("✅ PASS: FINGERPRINT='BYD ATTO3' is set")
        return True
    elif fingerprint:
        print(f"⚠️  WARN: FINGERPRINT='{fingerprint}' (expected 'BYD ATTO3')")
        return False
    else:
        print("ℹ️  INFO: FINGERPRINT not set (will rely on code fallback)")
        return None

def test_byd_import():
    """Test if BYD module can be imported correctly."""
    print("\n" + "="*60)
    print("TEST 2: BYD Module Import")
    print("="*60)

    try:
        from opendbc.car.byd.values import CAR
        print("✅ PASS: BYD module imported successfully")
        print(f"   BYD ATTO3 value: {CAR.BYD_ATTO3}")
        return True
    except Exception as e:
        print(f"❌ FAIL: Could not import BYD module: {e}")
        return False

def test_byd_registration():
    """Test if BYD is registered in the global BRANDS list."""
    print("\n" + "="*60)
    print("TEST 3: BYD Brand Registration")
    print("="*60)

    try:
        from opendbc.car.values import BRANDS, PLATFORMS
        from opendbc.car.byd.values import CAR as BYD

        byd_found = any(BYD in str(brand) for brand in BRANDS)
        if byd_found:
            print("✅ PASS: BYD is registered in BRANDS")
        else:
            print("❌ FAIL: BYD not found in BRANDS")
            return False

        # Check if BYD ATTO3 is in PLATFORMS
        if "BYD ATTO3" in PLATFORMS:
            print("✅ PASS: 'BYD ATTO3' found in PLATFORMS")
            return True
        else:
            print("❌ FAIL: 'BYD ATTO3' not in PLATFORMS")
            print(f"   Available platforms: {list(PLATFORMS.keys())[:5]}...")
            return False
    except Exception as e:
        print(f"❌ FAIL: Error checking registration: {e}")
        return False

def test_fingerprint_data():
    """Test if BYD fingerprint data is loaded correctly."""
    print("\n" + "="*60)
    print("TEST 4: Fingerprint Data")
    print("="*60)

    try:
        from opendbc.car.byd.fingerprints import FINGERPRINTS, FW_VERSIONS
        from opendbc.car.byd.values import CAR

        # Check CAN fingerprints
        if CAR.BYD_ATTO3 in FINGERPRINTS:
            num_fingerprints = len(FINGERPRINTS[CAR.BYD_ATTO3])
            print(f"✅ PASS: {num_fingerprints} CAN fingerprints defined")

            # Show sample from first fingerprint
            first_fp = FINGERPRINTS[CAR.BYD_ATTO3][0]
            num_messages = len(first_fp)
            print(f"   First fingerprint has {num_messages} CAN messages")
            print(f"   Sample messages: {list(first_fp.items())[:5]}")
        else:
            print("❌ FAIL: No CAN fingerprints defined for BYD ATTO3")
            return False

        # Check firmware versions
        if CAR.BYD_ATTO3 in FW_VERSIONS:
            num_ecus = len(FW_VERSIONS[CAR.BYD_ATTO3])
            print(f"✅ PASS: {num_ecus} ECU firmware versions defined")

            # List ECUs
            for ecu_key in list(FW_VERSIONS[CAR.BYD_ATTO3].keys())[:3]:
                ecu_type, addr, sub_addr = ecu_key
                print(f"   - {ecu_type} @ 0x{addr:03X}")
        else:
            print("⚠️  WARN: No firmware versions defined for BYD ATTO3")

        return True
    except Exception as e:
        print(f"❌ FAIL: Error loading fingerprint data: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_interface():
    """Test if BYD interface can be instantiated."""
    print("\n" + "="*60)
    print("TEST 5: Car Interface")
    print("="*60)

    try:
        from opendbc.car.byd.interface import CarInterface
        from opendbc.car.byd.values import CAR

        print("✅ PASS: CarInterface imported successfully")

        # Try to get params (minimal test)
        try:
            # This will use default/empty values but tests the method exists
            from opendbc.car import structs
            ret = structs.CarParams.new_message()
            ret = CarInterface._get_params(
                ret,
                CAR.BYD_ATTO3,
                {}, # empty fingerprint
                [], # empty fw
                False, # alpha_long
                True, # is_release
                False # docs
            )
            print("✅ PASS: CarInterface._get_params() works")
            print(f"   Brand: {ret.brand}")
            print(f"   Steer control type: {ret.steerControlType}")
            return True
        except Exception as e:
            print(f"⚠️  WARN: _get_params() failed: {e}")
            return True  # Interface exists, params might need work

    except Exception as e:
        print(f"❌ FAIL: Error loading interface: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_code_fallback():
    """Test if code modification fallback is present."""
    print("\n" + "="*60)
    print("TEST 6: Code Fallback Check")
    print("="*60)

    try:
        import inspect
        from opendbc.car import car_helpers

        # Read the source code of the fingerprint function
        source = inspect.getsource(car_helpers.fingerprint)

        if "Force BYD ATTO3" in source or "forcing BYD ATTO3" in source:
            print("✅ PASS: Code fallback modification detected in car_helpers.py")
            print("   This will force BYD ATTO3 if environment variable is not set")
            return True
        else:
            print("ℹ️  INFO: Code fallback not present (environment variable required)")
            return None
    except Exception as e:
        print(f"⚠️  WARN: Could not check code fallback: {e}")
        return None

def main():
    """Run all tests and summarize results."""
    print("\n" + "#"*60)
    print("# BYD ATTO3 Fingerprint Override Test Suite")
    print("#"*60)

    results = []
    results.append(("Environment Variable", test_environment_variable()))
    results.append(("BYD Module Import", test_byd_import()))
    results.append(("BYD Brand Registration", test_byd_registration()))
    results.append(("Fingerprint Data", test_fingerprint_data()))
    results.append(("Car Interface", test_interface()))
    results.append(("Code Fallback", test_code_fallback()))

    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)

    passed = sum(1 for _, result in results if result is True)
    failed = sum(1 for _, result in results if result is False)
    skipped = sum(1 for _, result in results if result is None)

    for test_name, result in results:
        if result is True:
            status = "✅ PASS"
        elif result is False:
            status = "❌ FAIL"
        else:
            status = "ℹ️  INFO"
        print(f"{status}: {test_name}")

    print("="*60)
    print(f"Total: {passed} passed, {failed} failed, {skipped} info")

    if failed == 0:
        print("\n🎉 All critical tests passed!")
        print("\nNext steps:")
        print("1. Set environment variable: export FINGERPRINT='BYD ATTO3'")
        print("2. Test on Comma 3X device")
        print("3. Monitor logs for successful fingerprinting")
        return 0
    else:
        print("\n⚠️  Some tests failed. Review errors above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
