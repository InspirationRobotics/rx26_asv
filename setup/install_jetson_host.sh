#!/usr/bin/env bash
# ============================================================================
# setup/install_jetson_host.sh — JETSON HOST setup (outside the container)
#
# Run ONCE per Jetson (and re-run after cabling changes). Installs the
# host-side plumbing the boot chain depends on, in dependency order:
#   udev rules -> systemd units (MAVProxy, container, LiDAR, power) ->
#   container mount check -> verify.
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

echo "== [1/5] udev rules (stable /dev/crsd-* symlinks + OAK-D perms) =="
bash tools/udev/install_udev.sh

echo "== [2/5] systemd units (crsd-mavproxy = sole Pixhawk owner, then container) =="
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
# crsd-power is the ground station's shutdown/reboot path: a container has no
# init of its own to ask, so this root helper on the HOST is what actually
# powers the machine off. See tools/scripts/crsd_power_helper.py.
for unit in crsd-mavproxy crsd-container crsd-livox crsd-power; do
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
systemctl enable crsd-mavproxy.service crsd-container.service crsd-livox.service crsd-power.service
echo "   enabled; start now with:"
echo "     systemctl start crsd-mavproxy crsd-container crsd-livox crsd-power"

echo "== [3/5] container mounts (recordings must outlive the container) =="
# crsd-container.service runs `docker start`, which CANNOT add mounts — they are
# fixed when the container is CREATED. So this step only checks, and tells you
# the exact docker run line if something is missing. Recreating the container is
# your call, not a script's: `docker rm` throws away anything living only inside
# it, and doing that unasked during a setup run is how a session disappears.
WS_HOST="$(dirname "$(dirname "$CRSD_REPO")")"     # ~/robotx_ws from .../src/rx26_asv
POWER_SOCK="${CRSD_POWER_SOCKET:-/run/crsd-power.sock}"
if docker inspect "$CRSD_CONTAINER" >/dev/null 2>&1; then
  MOUNT_LIST="$(docker inspect -f '{{range .Mounts}}{{.Source}}:{{.Destination}} {{end}}' "$CRSD_CONTAINER" 2>/dev/null || true)"
  MISSING=""
  case "$MOUNT_LIST" in *":/root/robotx_ws"*) ;; *) MISSING="$MISSING workspace" ;; esac
  case "$MOUNT_LIST" in *"$POWER_SOCK"*) ;; *) MISSING="$MISSING power-socket" ;; esac
  if [[ -n "$MISSING" ]]; then
    echo "WARN: container '$CRSD_CONTAINER' is MISSING mounts:$MISSING"
    echo "      Consequences:"
    echo "        workspace     -> ground station recordings live only inside the"
    echo "                         container and are DESTROYED by 'docker rm'."
    echo "        power-socket  -> the System tab cannot shut the Jetson down;"
    echo "                         it will show 'power helper unreachable'."
    echo "      To fix, recreate the container with BOTH mounts (adjust the rest"
    echo "      of the flags to match how yours was built):"
    echo
    echo "        docker rm -f $CRSD_CONTAINER"
    echo "        docker run -d --name $CRSD_CONTAINER \\"
    echo "          --restart unless-stopped --network host --privileged \\"
    echo "          -v $WS_HOST:/root/robotx_ws \\"
    echo "          -v $POWER_SOCK:$POWER_SOCK \\"
    echo "          -v /dev:/dev \\"
    echo "          $CRSD_CONTAINER tail -f /dev/null"
    echo
    echo "      Then re-run setup/install_container.sh inside it."
  else
    echo "   mounts OK: workspace and power socket are both bind-mounted."
  fi
else
  echo "WARN: no container named '$CRSD_CONTAINER' yet — create it with the two"
  echo "      bind mounts it needs:  -v $WS_HOST:/root/robotx_ws"
  echo "                             -v $POWER_SOCK:$POWER_SOCK"
fi

echo "== [4/5] host sanity checks =="
command -v docker >/dev/null || { echo "ERROR: docker not installed" >&2; exit 1; }
docker image inspect "$CRSD_CONTAINER" >/dev/null 2>&1 \
  || echo "WARN: no '$CRSD_CONTAINER' image found — build it (docker build -t $CRSD_CONTAINER .) before boot."
docker inspect "$CRSD_LIVOX_CONTAINER" >/dev/null 2>&1 \
  || echo "WARN: no container named '$CRSD_LIVOX_CONTAINER' — crsd-livox.service will fail. Set CRSD_LIVOX_CONTAINER and re-run."
command -v mavproxy.py >/dev/null \
  || echo "WARN: mavproxy.py not on PATH — crsd-mavproxy.service will fail."

echo "== [5/5] next step =="
echo "After a reboot (or starting the units), run preflight INSIDE the container:"
echo "    docker exec -it $CRSD_CONTAINER python3 /root/robotx_ws/src/rx26_asv/tools/scripts/preflight.py"
echo "Exit nonzero = do not arm. Then follow docs/SETUP_GUIDE.md §B.3."
