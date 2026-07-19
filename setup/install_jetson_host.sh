#!/usr/bin/env bash
# ============================================================================
# setup/install_jetson_host.sh — JETSON HOST setup (outside the container)
#
# Run ONCE per Jetson (and re-run after cabling changes). Installs the
# host-side plumbing the boot chain depends on, in dependency order:
#   udev rules -> systemd units (MAVProxy first, then container) -> verify.
#
# Prereqs: repo cloned to ~/robotx_ws, Docker + the `crusader` container image
# present (container build is the team's existing image; this script does not
# build it), MAVProxy installed on the host.
#
# Usage:   sudo bash setup/install_jetson_host.sh
# ============================================================================
set -euo pipefail
cd "$(dirname "$0")/.."   # repo root (= ~/robotx_ws on the Jetson)

if [[ "$(id -u)" -ne 0 ]]; then
  echo "ERROR: must run as root (udev + systemd installs)." >&2
  exit 1
fi

echo "== [1/4] udev rules (stable /dev/crsd-* symlinks + OAK-D perms) =="
bash tools/udev/install_udev.sh

echo "== [2/4] systemd units (crsd-mavproxy = sole Pixhawk owner, then container) =="
install -m 644 tools/systemd/crsd-mavproxy.service  /etc/systemd/system/
install -m 644 tools/systemd/crsd-container.service /etc/systemd/system/
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
echo "    docker exec -it crusader python3 /root/robotx_ws/tools/scripts/preflight.py"
echo "Exit nonzero = do not arm. Then follow docs/SETUP_GUIDE.md §B.3."
