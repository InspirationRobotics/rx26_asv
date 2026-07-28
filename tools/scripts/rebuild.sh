#!/usr/bin/env bash
# The one blessed rebuild path for Crusader code changes.
# The container COPIES files at build time — every edit requires this before it
# takes effect. "The change did nothing" almost always means this was skipped.
# Used identically by humans and by the Level-2 validate-and-revert step.
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
# Build only this repo's packages. The workspace may hold other package sources
# (e.g. the robotx_2026 boat repo) — rebuilding those is not this script's job,
# and their build failures must not block ours. colcon still errors if either
# selected package is missing, so a discovery regression fails loudly.
docker exec "$CONTAINER" bash -lc \
  "cd $WS && colcon build --symlink-install --packages-select interfaces rx26_asv"

echo "== import smoke test =="
# Fail loudly if any package doesn't import — a silently-inactive mechanism is a
# safety issue on this boat, not a nuisance.
docker exec "$CONTAINER" bash -lc \
  "cd $WS && source install/setup.bash && python3 -c 'import rx26_asv; print(\"import ok\")'"

echo "== done. Restart affected nodes/launch for changes to take effect. =="
