#!/usr/bin/env bash
# ============================================================================
# setup/install_container.sh — CONTAINER-SIDE setup (inside `asv`)
#
# Run once after the repo lands in the container mount, and re-run whenever
# package files change. Order matters:
#   dep guard -> colcon build -> import smoke.
#
# This script installs NOTHING. Runtime dependencies live in the image
# (see Dockerfile) so they are versioned, pinned, and survive `docker rm`.
# An unpinned `pip install` from here once resolved a dependency out from under
# the rest of the stack on the real Jetson — hence the guard below instead of an
# install step.
#
# Usage (from the Jetson host):
#     docker exec -it asv bash /root/robotx_ws/src/rx26_asv/setup/install_container.sh
# Or inside the container:
#     bash /root/robotx_ws/src/rx26_asv/setup/install_container.sh
# ============================================================================
set -euo pipefail

# This repo is ONE package source inside a colcon workspace — it is not the
# workspace root. Both paths are derived from this script's own location so a
# clone at a different path still works.
REPO="$(cd "$(dirname "$0")/.." && pwd)"      # repo root  (…/src/rx26_asv)
WS="${WS:-$(cd "$REPO/../.." && pwd)}"        # workspace root (…/robotx_ws)

if [[ ! -d "$WS/src" ]]; then
  echo "ERROR: '$WS' is not a colcon workspace (no src/)." >&2
  echo "       This repo must be cloned to <workspace>/src/rx26_asv," >&2
  echo "       or run with WS=/path/to/workspace." >&2
  exit 1
fi
echo "   repo:      $REPO"
echo "   workspace: $WS"

echo "== [1/3] Dependency guard (the image supplies these — we never install) =="
missing=0
for mod in yaml serial pymavlink cv_bridge torch; do
  python3 -c "import $mod" 2>/dev/null || { echo "   MISSING: $mod" >&2; missing=1; }
done
if [[ "$missing" -ne 0 ]]; then
  echo "ERROR: the container image is missing runtime deps." >&2
  echo "       Rebuild the image (docker build -t asv .) rather than" >&2
  echo "       pip-installing here — see the Dockerfile comment on pinning." >&2
  exit 1
fi

echo "== [2/3] colcon build (this repo's packages only) =="
# --packages-up-to crusader_bringup, not a bare build: the workspace may hold
# other package sources (the robotx_2026 boat repo, the sensor container's
# sources) whose build state is not ours to change and whose build failure must
# not block ours. crusader_bringup exec_depends on every package we ship, so
# "up-to" is the whole stack — and it stays correct when a package is added,
# which an explicit --packages-select list does not.
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to crusader_bringup
source install/setup.bash

echo "== [3/3] Import smoke (fail loudly — see tools/scripts/rebuild.sh) =="
python3 -c "import crusader_common, crusader_fcu, crusader_behavior; print('code packages ok')"
python3 -c "import crusader_perception, crusader_world_model; print('domain packages ok')"
python3 -c "from crusader_msgs.msg import FcuStatus; print('crusader_msgs ok')"
# The params file must be reachable from the INSTALL space, not just the source
# tree — resolving it is the failure that grounded the stack on 2026-07-29.
python3 -c "from crusader_common import config; print('params resolved:', config.DEFAULT_CONFIG_PATH)"
ros2 pkg executables crusader_fcu crusader_behavior || true

echo
echo "Done. For day-to-day edits use tools/scripts/rebuild.sh (host side),"
echo "which runs the same build + smoke via docker exec."
