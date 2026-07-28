#!/usr/bin/env bash
# The one blessed rebuild path for Crusader code changes.
# The container COPIES files at build time — every edit requires this before it
# takes effect. "The change did nothing" almost always means this was skipped.
# Used identically by humans and by the Level-2 validate-and-revert step.
set -euo pipefail

CONTAINER="crusader"
# Colcon WORKSPACE root inside the container (not the repo root — this repo is
# cloned to $WS/src/rx26_asv alongside any other package sources).
WS="${WS:-/root/robotx_ws}"

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  echo "ERROR: '$CONTAINER' container is not running." >&2
  exit 1
fi

echo "== colcon build inside $CONTAINER =="
# The repo root is deliberately NOT a colcon package, so normal discovery under
# src/ finds both rx26_asv and interfaces. If you ever see only one package
# built, that is a real error — do not paper over it with --base-paths.
docker exec "$CONTAINER" bash -lc \
  "cd $WS && colcon build --symlink-install"

echo "== import smoke test =="
# Fail loudly if any package doesn't import — a silently-inactive mechanism is a
# safety issue on this boat, not a nuisance.
docker exec "$CONTAINER" bash -lc \
  "cd $WS && source install/setup.bash && python3 -c 'import rx26_asv; print(\"import ok\")'"

echo "== done. Restart affected nodes/launch for changes to take effect. =="
