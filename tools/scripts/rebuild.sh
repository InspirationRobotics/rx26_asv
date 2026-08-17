#!/usr/bin/env bash
# The one blessed rebuild path for Crusader code changes.
# The container COPIES files at build time — every edit requires this before it
# takes effect. "The change did nothing" almost always means this was skipped.
set -euo pipefail

CONTAINER="${CRSD_CONTAINER:-asv}"
# Colcon WORKSPACE root inside the container (not the repo root — this repo is
# cloned to $WS/src/rx26_asv alongside any other package sources).
WS="${WS:-/root/robotx_ws}"

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  echo "ERROR: '$CONTAINER' container is not running." >&2
  exit 1
fi

echo "== colcon build inside $CONTAINER =="
# --packages-up-to crusader_bringup, not a bare build: the workspace may hold
# other package sources (the robotx_2026 boat repo, the livox container's
# sources) whose build state is not ours to change and whose build failure must
# not block ours. crusader_bringup exec_depends on every package we ship, so
# "up-to" is the whole stack — and it stays correct when a package is added,
# which an explicit --packages-select list does not.
docker exec "$CONTAINER" bash -lc \
  "cd $WS && colcon build --symlink-install --packages-up-to crusader_bringup"

echo "== import smoke test =="
# Fail loudly if any package doesn't import — a silently-inactive mechanism is a
# safety issue on this boat, not a nuisance. Every package with code is named
# here: a build that succeeds while an import fails is the exact gap this closes.
docker exec "$CONTAINER" bash -lc \
  "cd $WS && source install/setup.bash && python3 -c '
import crusader_common, crusader_fcu, crusader_behavior
import crusader_perception, crusader_world_model, crusader_groundstation
from crusader_msgs.msg import Attitude, FcuStatus, LatLonHead, RcChannels
from crusader_msgs.msg import Detection3D, Detection3DArray
from crusader_msgs.msg import Cluster3D, Cluster3DArray
from crusader_msgs.msg import TrackedTarget, TrackedTargetArray
print(\"import ok\")'"

echo "== done. Restart affected nodes/launch for changes to take effect. =="
