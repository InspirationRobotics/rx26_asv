#!/usr/bin/env bash
# Install Crusader udev rules on the Jetson host (NOT inside the container).
# Also bumps usbfs memory for the OAK-D LR (required for large stereo frames on USB3).
set -euo pipefail
# Unmatched globs expand to nothing rather than to the literal pattern, so the
# device-scan loops below behave when no serial devices are plugged in.
shopt -s nullglob

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
    # DIAGNOSTIC ONLY — this must never abort the install. grep exits 1 when a
    # device exposes none of these attributes (or when udevadm itself fails),
    # and under `set -e` + `pipefail` that killed the whole script HERE, before
    # a single rule was installed — silently, since set -e prints nothing.
    # That is the opposite of this block's stated intent two lines below.
    info=$(udevadm info -a -n "$dev" 2>/dev/null \
             | grep -m3 -E 'idVendor|idProduct|\{serial\}' \
             | tr -d ' ' | paste -sd' ' - || true)
    echo "  $dev  ${info:-(no usb attributes readable)}"
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
# NOT `ls -l /dev/crsd-*`: with nullglob an unmatched glob expands to NOTHING,
# so ls would get no arguments and cheerfully list the current directory —
# which reads as success. Collect into an array and test it.
crsd_links=(/dev/crsd-*)
if (( ${#crsd_links[@]} )); then
  ls -l "${crsd_links[@]}"
else
  echo "  (none yet — plug/replug devices, or the rules match no attached device)"
fi
