#!/usr/bin/env bash
# ============================================================================
# setup/install_container.sh — CONTAINER-SIDE setup (inside `crusader`)
#
# Run once after the repo lands in the container mount, and re-run whenever
# proto/ or package files change. Order matters:
#   pip deps -> protobuf compile -> colcon build (both packages) -> import smoke.
#
# Usage (from the Jetson host):
#     docker exec -it crusader bash /root/robotx_ws/src/rx26_asv/setup/install_container.sh
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

echo "== [1/4] Python deps not in the base image =="
# base image (ultralytics jetson-jetpack6) already ships CUDA/PyTorch/TensorRT,
# depthai and MAVProxy per CLAUDE.md — only top up the small pure-python bits.
pip install --no-cache-dir pyyaml protobuf grpcio-tools pymavlink pytest

echo "== [2/4] Compile RoboCommand protobuf (proto/ -> rx26_asv/api/mission/) =="
# Switches robocomms.py from its JSON fallback framing to real protobuf.
# Generated *_pb2.py files are .gitignored — always regenerated here.
# Paths are repo-relative: <repo>/rx26_asv is the package, and the python
# module dir is nested one deeper.
cd "$REPO"
python3 -m grpc_tools.protoc -I proto \
    --python_out=rx26_asv/rx26_asv/api/mission proto/robocommand.proto \
  || protoc -I proto --python_out=rx26_asv/rx26_asv/api/mission proto/robocommand.proto

echo "== [3/4] colcon build (both packages, from the workspace root) =="
# Repo root is deliberately NOT a package, so plain discovery under src/ finds
# rx26_asv AND interfaces — no --base-paths workaround, nothing silently skipped.
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

echo "== [4/4] Import smoke (fail loudly — see tools/scripts/rebuild.sh) =="
python3 -c "import rx26_asv; print('rx26_asv import ok')"
python3 -c "from interfaces.msg import Occupancy; print('interfaces msgs ok')"
ros2 pkg executables rx26_asv || true

echo
echo "Done. For day-to-day edits use tools/scripts/rebuild.sh (host side),"
echo "which runs the same build + smoke via docker exec."
