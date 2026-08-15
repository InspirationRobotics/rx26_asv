#!/usr/bin/env bash
# ============================================================================
# setup/install_jetson_host.sh — JETSON HOST setup (outside the container)
#
# Run ONCE per Jetson (and re-run after cabling changes). Installs the
# host-side plumbing the boot chain depends on, in dependency order:
#   udev rules -> systemd units (MAVProxy first, then container) -> verify.
#
# Prereqs: repo cloned to ~/robotx_ws/src/rx26_asv (it is one package source in
# the colcon workspace, not the workspace root), Docker + the `asv` container
# image present (build it from this repo's Dockerfile: `docker build -t asv .`;
# this script does not build it), MAVProxy installed on the host.
#
# Usage:   sudo bash setup/install_jetson_host.sh
# ============================================================================
set -euo pipefail
# Fail LOUDLY. `set -e` aborts with no message at all, so a step that dies
# halfway leaves a partially-installed boot chain looking like a clean run —
# this exact script once stopped after [1/4] with nothing but a warning on
# screen, and no systemd units were written.
trap 'rc=$?; echo >&2; echo "ERROR: install_jetson_host.sh ABORTED at line $LINENO (exit $rc)." >&2; echo "       The install is INCOMPLETE — no step after this one ran." >&2; echo "       Fix the cause and re-run; the script is idempotent." >&2' ERR
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
# Container name is a deployment choice, not a constant — override to stage a
# replacement image (e.g. CRSD_CONTAINER=asv-next) without editing units.
CRSD_CONTAINER="${CRSD_CONTAINER:-asv}"
# The MID360's container is NOT ours — it is the second container on the Jetson
# (docs/OPERATIONS.md §13) and its name is a deployment fact, not a constant.
CRSD_LIVOX_CONTAINER="${CRSD_LIVOX_CONTAINER:-crusader_legacy}"
id -u "$CRSD_USER" >/dev/null 2>&1 || {
  echo "ERROR: user '$CRSD_USER' does not exist — cannot install units." >&2
  echo "       Run with sudo from that user's session, or set SUDO_USER." >&2
  exit 1; }
echo "   service user:    $CRSD_USER"
echo "   repo path:       $CRSD_REPO"
echo "   container:       $CRSD_CONTAINER"
echo "   livox container: $CRSD_LIVOX_CONTAINER"
for unit in crsd-mavproxy crsd-container crsd-livox; do
  sed -e "s|__CRSD_USER__|$CRSD_USER|g" -e "s|__CRSD_REPO__|$CRSD_REPO|g" \
      -e "s|__CRSD_CONTAINER__|$CRSD_CONTAINER|g" \
      -e "s|__CRSD_LIVOX_CONTAINER__|$CRSD_LIVOX_CONTAINER|g" \
      "tools/systemd/$unit.service" > "/etc/systemd/system/$unit.service"
  chmod 644 "/etc/systemd/system/$unit.service"
  # Fail loudly rather than enabling a unit that still carries a placeholder.
  if grep -q "__CRSD_" "/etc/systemd/system/$unit.service"; then
    echo "ERROR: $unit.service still has unsubstituted placeholders." >&2; exit 1
  fi
done
systemctl daemon-reload
systemctl enable crsd-mavproxy.service crsd-container.service crsd-livox.service
echo "   enabled; start now with:"
echo "     systemctl start crsd-mavproxy crsd-container crsd-livox"

echo "== [3/4] host sanity checks =="
command -v docker >/dev/null || { echo "ERROR: docker not installed" >&2; exit 1; }
docker image inspect "$CRSD_CONTAINER" >/dev/null 2>&1 \
  || echo "WARN: no '$CRSD_CONTAINER' image found — build it (docker build -t $CRSD_CONTAINER .) before boot."
docker inspect "$CRSD_LIVOX_CONTAINER" >/dev/null 2>&1 \
  || echo "WARN: no container named '$CRSD_LIVOX_CONTAINER' — crsd-livox.service will fail. Set CRSD_LIVOX_CONTAINER and re-run."
command -v mavproxy.py >/dev/null \
  || echo "WARN: mavproxy.py not on PATH — crsd-mavproxy.service will fail."

echo "== [4/4] next step =="
echo "After a reboot (or starting the units), run preflight INSIDE the container:"
echo "    docker exec -it $CRSD_CONTAINER python3 /root/robotx_ws/src/rx26_asv/tools/scripts/preflight.py"
echo "Exit nonzero = do not arm. Then follow docs/SETUP_GUIDE.md §B.3."
