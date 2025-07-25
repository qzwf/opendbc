#!/usr/bin/env python3
"""
Simple test script to debug BYD import issues
"""
import traceback

def test_step(step_name, import_func):
    print(f"Testing {step_name}...")
    try:
        import_func()
        print(f"✓ {step_name} successful")
        return True
    except Exception as e:
        print(f"✗ {step_name} failed: {e}")
        traceback.print_exc()
        return False

def test_basic_imports():
    pass

def test_car_imports():
    pass

def test_fw_query_imports():
    pass

def test_byd_values():
    pass

def test_byd_fw_config():
    pass

def test_byd_fingerprints():
    pass

def main():
    print("BYD Import Test")
    print("=" * 50)

    tests = [
        ("Basic imports", test_basic_imports),
        ("Car imports", test_car_imports),
        ("FW query imports", test_fw_query_imports),
        ("BYD values", test_byd_values),
        ("BYD FW config", test_byd_fw_config),
        ("BYD fingerprints", test_byd_fingerprints),
    ]

    for step_name, test_func in tests:
        if not test_step(step_name, test_func):
            print(f"Stopping at failed step: {step_name}")
            return False

    print("\nAll tests passed!")
    return True

if __name__ == "__main__":
    main()
