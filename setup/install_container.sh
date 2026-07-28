#!/usr/bin/env bash
# ============================================================================
# setup/install_container.sh — CONTAINER-SIDE setup (inside `asv`)
#
# Run once after the repo lands in the container mount, and re-run whenever
# proto/ or package files change. Order matters:
#   dep guard -> protobuf compile -> colcon build -> import smoke.
#
# This script installs NOTHING. Runtime dependencies live in the image
# (see Dockerfile) so they are versioned, pinned, and survive `docker rm`.
# An unpinned `pip install` from here once resolved protobuf out from under
# TensorFlow on the real Jetson and broke the perception stack — hence the
# guard below instead of an install step.
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

echo "== [1/4] Dependency guard (the image supplies these — we never install) =="
missing=0
for mod in yaml google.protobuf pymavlink; do
  python3 -c "import $mod" 2>/dev/null || { echo "   MISSING: $mod" >&2; missing=1; }
done
if [[ "$missing" -ne 0 ]]; then
  echo "ERROR: the container image is missing runtime deps." >&2
  echo "       Rebuild the image (docker build -t asv .) rather than" >&2
  echo "       pip-installing here — see the Dockerfile comment on pinning." >&2
  exit 1
fi
python3 - <<'PY'
import google.protobuf as p
v = p.__version__
assert v.startswith("5.29"), (
    "protobuf is %s; the ultralytics base needs 5.29.x for TensorFlow "
    "(<6.0.0dev). Something moved it — rebuild the image." % v)
print("   protobuf", v, "ok")
PY

echo "== [2/4] Compile RoboCommand protobuf (proto/ -> rx26_asv/api/mission/) =="
# Switches robocomms.py from its JSON fallback framing to real protobuf.
# Generated *_pb2.py files are .gitignored — always regenerated here.
# Paths are repo-relative: <repo>/rx26_asv is the package, and the python
# module dir is nested one deeper.
cd "$REPO"
python3 -m grpc_tools.protoc -I proto \
    --python_out=rx26_asv/rx26_asv/api/mission proto/robocommand.proto \
  || protoc -I proto --python_out=rx26_asv/rx26_asv/api/mission proto/robocommand.proto

echo "== [3/4] colcon build (this repo's packages only) =="
# --packages-select, not a bare build: the workspace may hold other package
# sources (e.g. the robotx_2026 boat repo) whose build state is not ours to
# change, and whose build failure must not block ours. colcon errors if a
# selected package is missing, so a discovery regression still fails loudly.
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select interfaces rx26_asv
source install/setup.bash

echo "== [4/4] Import smoke (fail loudly — see tools/scripts/rebuild.sh) =="
python3 -c "import rx26_asv; print('rx26_asv import ok')"
python3 -c "from interfaces.msg import Occupancy; print('interfaces msgs ok')"
ros2 pkg executables rx26_asv || true

echo
echo "Done. For day-to-day edits use tools/scripts/rebuild.sh (host side),"
echo "which runs the same build + smoke via docker exec."
