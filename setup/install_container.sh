#!/usr/bin/env bash
# ============================================================================
# setup/install_container.sh — CONTAINER-SIDE setup (inside `crusader`)
#
# Run once after the repo lands in the container mount, and re-run whenever
# proto/ or package files change. Order matters:
#   pip deps -> protobuf compile -> colcon build (both packages) -> import smoke.
#
# Usage (from the Jetson host):
#     docker exec -it crusader bash /root/robotx_ws/setup/install_container.sh
# Or inside the container:
#     bash /root/robotx_ws/setup/install_container.sh
# ============================================================================
set -euo pipefail
WS="${WS:-/root/robotx_ws}"
cd "$WS"

echo "== [1/4] Python deps not in the base image =="
# base image (ultralytics jetson-jetpack6) already ships CUDA/PyTorch/TensorRT,
# depthai and MAVProxy per CLAUDE.md — only top up the small pure-python bits.
pip install --no-cache-dir pyyaml protobuf grpcio-tools pymavlink pytest

echo "== [2/4] Compile RoboCommand protobuf (proto/ -> rx26_asv/api/mission/) =="
# Switches robocomms.py from its JSON fallback framing to real protobuf.
# Generated *_pb2.py files are .gitignored — always regenerated here.
python3 -m grpc_tools.protoc -I proto \
    --python_out=rx26_asv/api/mission proto/robocommand.proto \
  || protoc -I proto --python_out=rx26_asv/api/mission proto/robocommand.proto

echo "== [3/4] colcon build (rx26_asv at root + interfaces/) =="
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths . interfaces
source install/setup.bash

echo "== [4/4] Import smoke (fail loudly — see tools/scripts/rebuild.sh) =="
python3 -c "import rx26_asv; print('rx26_asv import ok')"
python3 -c "from interfaces.msg import Occupancy; print('interfaces msgs ok')"
ros2 pkg executables rx26_asv || true

echo
echo "Done. For day-to-day edits use tools/scripts/rebuild.sh (host side),"
echo "which runs the same build + smoke via docker exec."
