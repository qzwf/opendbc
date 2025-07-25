from parameterized import parameterized

from opendbc.car.byd.fingerprints import FW_VERSIONS
from opendbc.car.byd.values import FW_QUERY_CONFIG
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu


class TestBYDFingerprint:
    @parameterized.expand(FW_VERSIONS.items())
    def test_fw_fingerprints(self, car_model, fw_versions):
        """Test that FW_VERSIONS contains valid firmware version data for each car model."""
        assert len(fw_versions) > 0, f"No firmware versions defined for {car_model}"

        # Check that each ECU entry has the correct structure
        for (ecu, addr, sub_addr), versions in fw_versions.items():
            assert isinstance(ecu, int), f"ECU must be an integer enum value: {ecu}"
            # Check that ECU value is within valid range (based on capnp enum values)
            assert 0 <= ecu <= 24, f"ECU enum value out of range: {ecu}"
            assert isinstance(addr, int), f"Invalid address type: {addr}"
            assert addr > 0, f"Address must be positive: {addr}"
            assert sub_addr is None or isinstance(sub_addr, int), f"Invalid sub_addr type: {sub_addr}"
            assert len(versions) > 0, f"No firmware versions for ECU {ecu} at address {addr:#x}"

            # Check that all firmware versions are bytes
            for version in versions:
                assert isinstance(version, bytes), f"Firmware version must be bytes: {version}"
                assert len(version) > 0, f"Empty firmware version for ECU {ecu}"

    def test_fw_query_config(self):
        """Test that FW_QUERY_CONFIG is properly configured."""
        assert FW_QUERY_CONFIG is not None, "FW_QUERY_CONFIG must be defined"
        assert len(FW_QUERY_CONFIG.requests) > 0, "FW_QUERY_CONFIG must have at least one request"

        # Check that extra_ecus contains the expected ECUs
        extra_ecu_addrs = {addr for (ecu, addr, sub_addr) in FW_QUERY_CONFIG.extra_ecus}

        # Verify that data collection ECU addresses are present (essential ECUs should NOT be in extra_ecus)
        expected_addrs = {0x7D0, 0x706, 0x1E2, 0x316, 0x32D, 0x32E,
                         0x320, 0x321, 0x322, 0x323}

        for addr in expected_addrs:
            assert addr in extra_ecu_addrs, f"Expected ECU address {addr:#x} not found in extra_ecus"

    def test_ecu_coverage(self):
        """Test that FW_VERSIONS covers the essential ECUs for fingerprinting."""
        # Get essential ECUs from non-logging requests in FW_QUERY_CONFIG
        essential_ecus = set()
        for request in FW_QUERY_CONFIG.requests:
            if not request.logging:  # Only non-logging requests are used for fingerprinting
                essential_ecus.update(request.whitelist_ecus)

        # Find the addresses for essential ECUs only
        essential_ecu_addrs = set()
        for config_ecu, addr, _sub_addr in FW_QUERY_CONFIG.extra_ecus:
            if config_ecu in essential_ecus:
                essential_ecu_addrs.add((config_ecu, addr))

        # Get ECUs from FW_VERSIONS
        for car_model, fw_versions in FW_VERSIONS.items():
            fw_ecu_addrs = {(ecu, addr) for (ecu, addr, sub_addr) in fw_versions.keys()}

            # Check that FW_VERSIONS covers the essential ECUs
            for (ecu, addr) in essential_ecu_addrs:
                assert (ecu, addr) in fw_ecu_addrs, \
                    f"Essential ECU {ecu} at address {addr:#x} missing from FW_VERSIONS for {car_model}"
