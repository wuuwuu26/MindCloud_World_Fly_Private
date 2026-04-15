#!/bin/bash
#
# setup_udev.sh — Grant non-root access to RC transmitter HID devices
#
# Run once with sudo:
#   sudo bash setup_udev.sh
#
# Covers RadioMaster, FrSky, Jumper, ELRS, and generic HID devices.

set -e

RULE_FILE="/etc/udev/rules.d/99-rc-transmitters.rules"

cat > "$RULE_FILE" << 'EOF'
# RC Transmitter HID access for regular users
# RadioMaster / OpenTX / EdgeTX (generic HID joystick)
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="1209", MODE="0666"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483", MODE="0666"
# FrSky
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666"
# Jumper
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="2341", MODE="0666"
# Generic: allow all hidraw devices for plugdev group
SUBSYSTEM=="hidraw", GROUP="plugdev", MODE="0660"
EOF

echo "Written: $RULE_FILE"

# Add current user to plugdev group (if not already)
TARGET_USER="${SUDO_USER:-$USER}"
if ! groups "$TARGET_USER" | grep -q plugdev; then
    usermod -aG plugdev "$TARGET_USER"
    echo "Added $TARGET_USER to plugdev group (re-login required)"
else
    echo "$TARGET_USER already in plugdev group"
fi

udevadm control --reload-rules
udevadm trigger

echo ""
echo "Done. Reconnect your RC transmitter and run launch.sh."
