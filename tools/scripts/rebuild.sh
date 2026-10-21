#!/usr/bin/env bash
# The one rebuild path for Crusader code changes, from the Jetson host: runs
# setup/install_container.sh (build + smoke) inside the container. Every edit needs
# it; "the change did nothing" almost always means it was skipped.
set -euo pipefail

CONTAINER="${CRSD_CONTAINER:-asv}"
WS="${WS:-/root/robotx_ws}"

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  echo "ERROR: '$CONTAINER' container is not running." >&2
  exit 1
fi
docker exec "$CONTAINER" env WS="$WS" bash "$WS/src/rx26_asv/setup/install_container.sh"
