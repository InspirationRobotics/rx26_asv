#!/bin/bash
# Run one ROS node inside the `asv` container, under systemd, without leaving a
# second copy behind on restart.
#
#     scripts/run_in_container.sh <package> <executable>
#
# ============================================================================
# THE PROBLEM THIS EXISTS TO SOLVE
#
# `docker exec` DOES NOT PROPAGATE TERMINATION. Kill the exec client and the
# process it started keeps running inside the container. So the obvious unit —
# ExecStart=docker exec asv ros2 run ... — behaves like this:
#
#     systemctl restart crsd-groundstation
#       -> systemd SIGTERMs the exec CLIENT, which dies
#       -> the node inside the container KEEPS RUNNING, still holding :8090
#       -> systemd starts a new client, which starts a SECOND node
#       -> the second dies with "address already in use"
#
# and the journal shows a crash, not a leftover. The fix is three layers,
# because each covers a case the others do not:
#
#   1. SWEEP ON THE WAY IN (below, and ExecStartPre). Kill any orphan BEFORE
#      starting. This is the one that matters: it works even when the PREVIOUS
#      stop failed — container was down, docker daemon was restarting, the unit
#      was SIGKILLed. A stop path that must always have worked is not something
#      to rely on; sweeping on entry is.
#   2. TRAP AND FORWARD (below). SIGTERM to this wrapper kills the process
#      INSIDE the container, so the normal systemd stop works rather than
#      depending on ExecStop being correct.
#   3. ExecStop in the unit. Backstop for the case where this script itself is
#      gone.
#
# THE KILL TARGET IS THE INSTALL-SPACE PATH, NOT THE NODE NAME. After `ros2 run`
# execs the node, the live process's argv is
#
#     /root/robotx_ws/install/crusader_groundstation/lib/crusader_groundstation/ground_station
#
# Matching on `ground_station` alone would also match a neighbouring node whose
# name contains it, plus any editor, tail or grep someone left open in the
# container. The install-space path cannot match anything else on the machine.
# The unit's ExecStartPre/ExecStop must use the SAME string — it is printed at
# startup so a mismatch is visible rather than silent.
# ============================================================================
set -euo pipefail

PKG="${1:?usage: run_in_container.sh <package> <executable>}"
EXE="${2:?usage: run_in_container.sh <package> <executable>}"

CONTAINER="${CRSD_CONTAINER:-asv}"
WS="${CRSD_WS:-/root/robotx_ws}"
ROS_DISTRO_SETUP="${CRSD_ROS_SETUP:-/opt/ros/humble/setup.bash}"

#: The one string that identifies this node's process inside the container.
MARKER="install/${PKG}/lib/${PKG}/${EXE}"

TERM_GRACE_S=5
CONTAINER_WAIT_S=60

log() { echo "[run_in_container] $*"; }

# ---- 1. wait for the container, bounded --------------------------------------
# crsd-container.service has run `docker start`, but the container may not be
# ready yet. Failing with a real message beats exec'ing into nothing and getting
# docker's own "No such container", which reads like a misconfiguration.
waited=0
# `container inspect`, not the bare form: the image is named `asv` too, and the
# bare form would match it and return an empty .State forever.
until [ "$(docker container inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ]; do
  if [ "$waited" -ge "$CONTAINER_WAIT_S" ]; then
    log "ERROR: container '$CONTAINER' is not running after ${CONTAINER_WAIT_S}s."
    log "       systemctl status crsd-container   # is it up?"
    log "       docker ps -a                     # does it exist at all?"
    exit 1
  fi
  sleep 1
  waited=$((waited + 1))
done
[ "$waited" -gt 0 ] && log "container '$CONTAINER' ready after ${waited}s"

# ---- 2. fail loudly on an unbuilt workspace ----------------------------------
# Without this the failure is `ros2: command not found` deep in a journal, which
# sends people looking at PATH and ROS installs instead of at the build.
if ! docker exec "$CONTAINER" test -f "$WS/install/setup.bash"; then
  log "ERROR: $WS/install/setup.bash is missing inside '$CONTAINER'."
  log "       The workspace has never been built. Run:"
  log "         docker exec $CONTAINER bash -lc '$WS/src/rx26_asv/tools/scripts/rebuild.sh'"
  exit 1
fi

# ---- 3. sweep orphans BEFORE starting ----------------------------------------
sweep() {
  local sig="$1"
  docker exec "$CONTAINER" pkill "-$sig" -f "$MARKER" 2>/dev/null || true
}
if docker exec "$CONTAINER" pgrep -f "$MARKER" >/dev/null 2>&1; then
  log "orphan(s) of $EXE already inside the container — sweeping before start"
  sweep TERM
  for _ in $(seq "$TERM_GRACE_S"); do
    docker exec "$CONTAINER" pgrep -f "$MARKER" >/dev/null 2>&1 || break
    sleep 1
  done
  if docker exec "$CONTAINER" pgrep -f "$MARKER" >/dev/null 2>&1; then
    log "orphan did not exit on SIGTERM after ${TERM_GRACE_S}s — SIGKILL"
    sweep KILL
    sleep 1
  fi
fi

# ---- 4. trap, so the NORMAL stop path works ----------------------------------
child=""
on_term() {
  log "stopping $EXE inside '$CONTAINER'"
  sweep TERM
  for _ in $(seq "$TERM_GRACE_S"); do
    docker exec "$CONTAINER" pgrep -f "$MARKER" >/dev/null 2>&1 || break
    sleep 1
  done
  docker exec "$CONTAINER" pgrep -f "$MARKER" >/dev/null 2>&1 && sweep KILL
  [ -n "$child" ] && kill "$child" 2>/dev/null || true
  exit 0
}
trap on_term TERM INT

# ---- 5. run it ---------------------------------------------------------------
# SIGTERM inside reaches crusader_common.node_main, which converts it into a normal
# exception so destroy_node() runs: thread joins, MAVLink close, OCS link close.
log "starting $PKG/$EXE  (marker: $MARKER)"
docker exec "$CONTAINER" bash -lc \
  "source '$ROS_DISTRO_SETUP' && source '$WS/install/setup.bash' && \
   exec ros2 run '$PKG' '$EXE'" &
child=$!

# `wait` returns immediately on a trapped signal, so loop until the child is
# genuinely gone. Without the loop, a SIGTERM would return from wait and fall
# off the end of the script while on_term was still tearing down.
wait "$child"
rc=$?
log "$EXE exited with $rc"
exit "$rc"
