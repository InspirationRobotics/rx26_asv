#!/usr/bin/env bash
# Install Crusader udev rules on the Jetson host (NOT inside the container).
# Also bumps usbfs memory for the OAK-D LR (required for large stereo frames on USB3).
set -euo pipefail

RULES_DIR="$(dirname "$0")"
RULES_SRC="$RULES_DIR/99-crusader.rules"

if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo." >&2
  exit 1
fi

# Help the operator fill in serials before installing.
if grep -q "TODO_GPS1_SERIAL" "$RULES_SRC"; then
  echo "WARNING: GPS serial placeholders not filled in. Plugged-in candidates:"
  for dev in /dev/ttyACM* /dev/ttyUSB*; do
    [[ -e "$dev" ]] || continue
    info=$(udevadm info -a -n "$dev" 2>/dev/null | grep -m3 -E 'idVendor|idProduct|\{serial\}' | tr -d ' ' | paste -sd' ' -)
    echo "  $dev  $info"
  done
  echo "Edit $RULES_SRC, then re-run. (Installing anyway so non-GPS rules take effect.)"
fi

# install the hand-maintained VID rules AND any generated per-boat rules
# (99-crusader-devpath.rules from gen_udev_rules.py + a boat config JSON)
for f in "$RULES_DIR"/99-crusader*.rules; do
  echo "installing $(basename "$f")"
  install -m 0644 "$f" "/etc/udev/rules.d/$(basename "$f")"
done

# OAK-D: usbfs memory (default 16MB is too small for stereo LR streams)
if ! grep -q "usbcore.usbfs_memory_mb" /etc/modprobe.d/usbfs.conf 2>/dev/null; then
  echo "options usbcore usbfs_memory_mb=1000" > /etc/modprobe.d/usbfs.conf
fi
# apply immediately without reboot
echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb || true

udevadm control --reload-rules
udevadm trigger

echo "Installed. Verify symlinks:"
ls -l /dev/crsd-* 2>/dev/null || echo "  (none yet — plug/replug devices or check serials)"
