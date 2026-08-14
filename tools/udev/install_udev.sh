#!/usr/bin/env bash
# Install Crusader udev rules on the Jetson host (NOT inside the container).
# Also bumps usbfs memory for the OAK-D LR. That camera belongs to a DIFFERENT
# container, but usbfs is a host-kernel setting and udev rules are host state:
# dropping them here would silently break the camera on a shared Jetson. Move
# both to the camera container's own installer when it grows one.
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

# List the serial devices present, so an unexpected one is visible at install
# time rather than discovered as a missing symlink later.
echo "attached serial devices:"
for dev in /dev/ttyACM* /dev/ttyUSB*; do
  [[ -e "$dev" ]] || continue
  # DIAGNOSTIC ONLY — this must never abort the install. grep exits 1 when a
  # device exposes none of these attributes (or when udevadm itself fails),
  # and under `set -e` + `pipefail` that killed the whole script HERE, before
  # a single rule was installed — silently, since set -e prints nothing.
  info=$(udevadm info -a -n "$dev" 2>/dev/null \
           | grep -m3 -E 'idVendor|idProduct|\{serial\}' \
           | tr -d ' ' | paste -sd' ' - || true)
  echo "  $dev  ${info:-(no usb attributes readable)}"
done

# Purge installed rules that no longer exist in the repo. `install` only ever
# ADDS files, so deleting a rules file from the repo left the old copy live in
# /etc/udev/rules.d forever — and re-running this script looked like it had
# resolved the problem. That is how 99-crusader-devpath.rules kept mapping
# crsd-ball-launcher onto the Pixhawk's tty after the repo dropped it: a stale
# port-chain rule outliving the cabling it described. Rules removed here, not
# just overwritten, so the collision guard below reflects the repo's intent.
for installed in /etc/udev/rules.d/99-crusader*.rules; do
  name="$(basename "$installed")"
  if [[ ! -e "$RULES_DIR/$name" ]]; then
    echo "removing stale $name (no longer in the repo)"
    rm -f "$installed"
  fi
done

# install every 99-crusader*.rules the repo carries (today: just the one
# hand-maintained VID/PID file)
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

# --- Collision guard -------------------------------------------------------
# Two rules claiming one device is silent and dangerous. A stale generated
# port-chain rule once pointed crsd-ball-launcher at the SAME tty as
# crsd-pixhawk, so a node opening that name would have opened the autopilot's
# serial link at 9600 baud while MAVProxy owned it — a second Pixhawk owner, on
# a boat with live thrusters. It was only caught by eyeballing an `ls`. The
# port-chain generator that produced that rule is gone, but the guard stays:
# the failure mode belongs to udev, not to that one script.
declare -A crsd_seen=()
collision=0
for link in "${crsd_links[@]}"; do
  target="$(readlink -f "$link")" || continue
  name="$(basename "$link")"
  if [[ -n "${crsd_seen[$target]:-}" ]]; then
    echo "ERROR: $name and ${crsd_seen[$target]} BOTH resolve to $target" >&2
    case "$name ${crsd_seen[$target]}" in
      *crsd-pixhawk*)
        echo "       This aliases the AUTOPILOT. Anything opening the other" >&2
        echo "       name becomes a second owner of the Pixhawk serial link." >&2 ;;
    esac
    collision=1
  else
    crsd_seen[$target]="$name"
  fi
done
if (( collision )); then
  echo >&2
  echo "ERROR: conflicting udev symlinks — refusing to report success." >&2
  echo "       Check for stale rules left by an earlier install:" >&2
  echo "         ls -l /etc/udev/rules.d/99-crusader*" >&2
  echo "       Two devices sharing a VID/PID (e.g. a second CH340, which has no" >&2
  echo "       serial number) will do this too — see 99-crusader.rules." >&2
  exit 1
fi
