#!/usr/bin/env bash
# The one blessed rebuild path for Crusader code changes.
# The container COPIES files at build time — every edit requires this before it
# takes effect. "The change did nothing" almost always means this was skipped.
# Used identically by humans and by the Level-2 validate-and-revert step.
set -euo pipefail

CONTAINER="crusader"
WS="/root/robotx_ws"

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  echo "ERROR: '$CONTAINER' container is not running." >&2
  exit 1
fi

echo "== colcon build inside $CONTAINER =="
# The repo root is itself the rx26_asv ament_python package (setup.py at
# root), so colcon will not descend into interfaces/ on its own — both base
# paths must be named explicitly.
docker exec "$CONTAINER" bash -lc \
  "cd $WS && colcon build --symlink-install --base-paths . interfaces"

echo "== import smoke test =="
# Fail loudly if any package doesn't import — a silently-inactive mechanism is a
# safety issue on this boat, not a nuisance.
docker exec "$CONTAINER" bash -lc \
  "cd $WS && source install/setup.bash && python3 -c 'import rx26_asv; print(\"import ok\")'"

echo "== done. Restart affected nodes/launch for changes to take effect. =="
