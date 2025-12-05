#!/bin/bash
#
# Force BYD ATTO3 Recognition Script
# This script sets environment variables to bypass automatic fingerprinting
# and force openpilot to recognize the vehicle as BYD ATTO3
#
# Usage:
#   On Comma 3X device: 
#     1. SSH into device: ssh comma@<device-ip>
#     2. Run: source /data/openpilot/opendbc_repo/force_byd_atto3.sh
#     3. Restart openpilot: reboot
#
#   On development machine:
#     source ./force_byd_atto3.sh

echo "=================================================="
echo "   BYD ATTO3 Forced Fingerprint Configuration"
echo "=================================================="

# Force fingerprint to BYD ATTO3
export FINGERPRINT="BYD ATTO3"

# Optional: Skip firmware query to speed up startup (not recommended for first runs)
# export SKIP_FW_QUERY=1

# Optional: Disable firmware cache to force fresh detection each time
# export DISABLE_FW_CACHE=1

echo "✓ FINGERPRINT set to: $FINGERPRINT"
echo ""
echo "Fingerprinting will be forced to BYD ATTO3"
echo "This bypasses automatic CAN/FW detection"
echo ""
echo "To make this permanent on Comma 3X:"
echo "  1. Add 'export FINGERPRINT=\"BYD ATTO3\"' to /data/bashrc"
echo "  2. Or add to openpilot launch script"
echo ""
echo "To test current fingerprinting:"
echo "  python3 -c \"import os; print('Current:', os.environ.get('FINGERPRINT', 'AUTO-DETECT'))\""
echo "=================================================="
