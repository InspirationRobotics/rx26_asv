#!/usr/bin/env bash
# ============================================================================
# setup/install_jetson_host.sh — JETSON HOST setup (outside the container)
#
# Run ONCE per Jetson (and re-run after cabling changes). Installs the
# host-side plumbing the boot chain depends on, in dependency order:
#   udev rules -> systemd units (MAVProxy first, then container) -> verify.
#
# Prereqs: repo cloned to ~/robotx_ws/src/rx26_asv (it is one package source in
# the colcon workspace, not the workspace root), Docker + the `crusader`
# container image present (container build is the team's existing image; this
# script does not build it), MAVProxy installed on the host.
#
# Usage:   sudo bash setup/install_jetson_host.sh
# ============================================================================
set -euo pipefail
cd "$(dirname "$0")/.."   # repo root (= ~/robotx_ws/src/rx26_asv on the Jetson)

if [[ "$(id -u)" -ne 0 ]]; then
  echo "ERROR: must run as root (udev + systemd installs)." >&2
  exit 1
fi

echo "== [1/4] udev rules (stable /dev/crsd-* symlinks + OAK-D perms) =="
bash tools/udev/install_udev.sh

echo "== [2/4] systemd units (crsd-mavproxy = sole Pixhawk owner, then container) =="
# The units are templates: the service account and repo path differ per Jetson,
# and a hardcoded /home/<someone> silently fails at boot — which you discover on
# the water, not at the bench. Substitute the real values at install time.
CRSD_USER="${SUDO_USER:-$USER}"
CRSD_REPO="$(pwd)"
id -u "$CRSD_USER" >/dev/null 2>&1 || {
  echo "ERROR: user '$CRSD_USER' does not exist — cannot install units." >&2
  echo "       Run with sudo from that user's session, or set SUDO_USER." >&2
  exit 1; }
echo "   service user: $CRSD_USER"
echo "   repo path:    $CRSD_REPO"
for unit in crsd-mavproxy crsd-container; do
  sed -e "s|__CRSD_USER__|$CRSD_USER|g" -e "s|__CRSD_REPO__|$CRSD_REPO|g" \
      "tools/systemd/$unit.service" > "/etc/systemd/system/$unit.service"
  chmod 644 "/etc/systemd/system/$unit.service"
  # Fail loudly rather than enabling a unit that still carries a placeholder.
  if grep -q "__CRSD_" "/etc/systemd/system/$unit.service"; then
    echo "ERROR: $unit.service still has unsubstituted placeholders." >&2; exit 1
  fi
done
systemctl daemon-reload
systemctl enable crsd-mavproxy.service crsd-container.service
echo "   enabled; start now with: systemctl start crsd-mavproxy crsd-container"

echo "== [3/4] host sanity checks =="
command -v docker >/dev/null || { echo "ERROR: docker not installed" >&2; exit 1; }
docker image inspect crusader >/dev/null 2>&1 \
  || echo "WARN: no 'crusader' image found — build/load the team image before boot."
command -v mavproxy.py >/dev/null \
  || echo "WARN: mavproxy.py not on PATH — crsd-mavproxy.service will fail."

echo "== [4/4] next step =="
echo "After a reboot (or starting the units), run preflight INSIDE the container:"
echo "    docker exec -it crusader python3 /root/robotx_ws/src/rx26_asv/tools/scripts/preflight.py"
echo "Exit nonzero = do not arm. Then follow docs/SETUP_GUIDE.md §B.3."
